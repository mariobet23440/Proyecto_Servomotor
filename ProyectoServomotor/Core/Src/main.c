/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Estructura para registrar eventos de scheduling con marca de tiempo
typedef struct {
    uint32_t timestamp; // Marca de tiempo en ms (HAL_GetTick())
    uint8_t event_id;   // ID del evento (ej: 1=ADC, 2=UART_TX, 3=UART_RX)
} LogEntry_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TELEMETRY_HEADER_1  0xAA
#define TELEMETRY_HEADER_2  0x55

// Tamaños máximos para el historial de scheduling
#define LOG_BUFFER_SIZE     20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

// Buffer DMA para los 2 canales del ADC:
// adc_buffer[0] -> Canal 0 (Sensor / Planta y_k)
// adc_buffer[1] -> Canal 1 (Referencia / Setpoint r_k)
uint16_t adc_buffer[2] = {0, 0};

volatile uint16_t dac_val = 0;

// Parámetros PID y Muestreo
float Ts = 0.001f; // 1 ms
float kP = 1.5f;
float kI = 0.05f;
float kD = 0.01f;

// Variables globales para la telemetría
volatile float g_e_k = 0.0f;
volatile float g_E_k = 0.0f;
volatile float g_eD  = 0.0f;

// Flags de telemetría
uint8_t flag_send_signals    = 1;
uint8_t flag_send_pid_params = 1;
uint8_t flag_send_scheduling = 1;

// Log de scheduling
LogEntry_t log_buffer[LOG_BUFFER_SIZE];
volatile uint8_t log_index = 0;

// Flag para saber si el DMA del UART está libre para transmitir
volatile uint8_t uart_tx_ready = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void TIM2_SetPeriod(float Ts);
uint16_t PID_Compute(uint16_t sensor_val, uint16_t ref_val, float kP, float kI, float kD, float Ts);
uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length);
void Log_Event(uint8_t event_id);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Configura el período de TIM2 para definir el tiempo de muestreo.
 * @param Ts Período objetivo en segundos (ej. 0.001f para 1 ms / 1 kHz)
 */
void TIM2_SetPeriod(float Ts)
{
    // Frecuencia de reloj del Timer 2 en APB1 (asumiendo ~84 MHz según SystemClock_Config)
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        timer_clk *= 2; // Si el prescalador APB1 != 1, la frecuencia del Timer es 2x PCLK1
    }

    // Obtener prescalador actual
    uint32_t prescaler = htim2.Instance->PSC;

    // Calcular el valor del registro ARR (Period): ARR = (f_clk * Ts) / (PSC + 1) - 1
    uint32_t arr_value = (uint32_t)((float)timer_clk * Ts / (float)(prescaler + 1)) - 1;

    // Actualizar el registro Auto-Reload de TIM2
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr_value);
}

/**
 * @brief Calcula la salida del PID digital dada la medición del sensor y la referencia.
 * @param sensor_val Valor leído por el ADC (0 - 4095)
 * @param ref_val Valor de referencia o Setpoint (0 - 4095)
 * @param kP Ganancia Proporcional
 * @param kI Ganancia Integral
 * @param kD Ganancia Derivativa
 * @param Ts Período de muestreo en segundos
 * @retval Valor a escribir en el DAC (uint16_t, rango 0 - 4095)
 */
uint16_t PID_Compute(uint16_t sensor_val, uint16_t ref_val, float kP, float kI, float kD, float Ts)
{
    static float e_k_1 = 0.0f;
    static float E_k_1 = 0.0f;

    float r_k = (float)ref_val;
    float y_k = (float)sensor_val;

    float e_k = r_k - y_k;
    float E_k = E_k_1 + e_k;
    float eD  = e_k - e_k_1;

    // Guardar variables globales para la telemetría
    g_e_k = e_k;
    g_E_k = E_k;
    g_eD  = eD;

    float u_k = (kP * e_k) + ((kI / Ts) * E_k) + ((kD / Ts) * eD);

    e_k_1 = e_k;
    E_k_1 = E_k;

    if (u_k > 4095.0f) {
        u_k = 4095.0f;
    } else if (u_k < 0.0f) {
        u_k = 0.0f;
    }

    return (uint16_t)u_k;
}

/**
 * @brief Calcula el CRC-8 de un buffer de datos (Polinomio 0x07 / ATM-8)
 */
uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Agrega un evento al buffer circular de marcas de tiempo
 */
void Log_Event(uint8_t event_id)
{
    log_buffer[log_index].timestamp = HAL_GetTick();
    log_buffer[log_index].event_id  = event_id;
    log_index = (log_index + 1) % LOG_BUFFER_SIZE;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 1. Ajustar el período de muestreo en TIM2
  TIM2_SetPeriod(Ts);

  // 2. Encender el módulo DAC
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  // 3. Iniciar el escaneo continuo de 2 canales del ADC con DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);

  // 4. Iniciar TIM2 para activar las conversiones periódicas
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	  // Transmitir solo si el canal DMA del UART está libre
	  if (uart_tx_ready)
	  {
		  static uint8_t tx_buffer[256];
		  uint16_t tx_idx = 0;

		  // --- Header ---
		  tx_buffer[tx_idx++] = TELEMETRY_HEADER_1;
		  tx_buffer[tx_idx++] = TELEMETRY_HEADER_2;

		  // --- BLOQUE 1: Señales Mínimas ---
		  if (flag_send_signals)
		  {
			  uint16_t ref_val    = adc_buffer[1];
			  uint16_t sensor_val = adc_buffer[0];
			  uint16_t u_k_val    = dac_val;

			  memcpy(&tx_buffer[tx_idx], &ref_val, sizeof(ref_val));
			  tx_idx += sizeof(ref_val);

			  memcpy(&tx_buffer[tx_idx], &sensor_val, sizeof(sensor_val));
			  tx_idx += sizeof(sensor_val);

			  memcpy(&tx_buffer[tx_idx], &u_k_val, sizeof(u_k_val));
			  tx_idx += sizeof(u_k_val);

			  float e_k_tmp = g_e_k;
			  float E_k_tmp = g_E_k;
			  float eD_tmp  = g_eD;

			  memcpy(&tx_buffer[tx_idx], &e_k_tmp, sizeof(float));
			  tx_idx += sizeof(float);

			  memcpy(&tx_buffer[tx_idx], &E_k_tmp, sizeof(float));
			  tx_idx += sizeof(float);

			  memcpy(&tx_buffer[tx_idx], &eD_tmp, sizeof(float));
			  tx_idx += sizeof(float);
		  }

		  // --- BLOQUE 2: Parámetros PID ---
		  if (flag_send_pid_params)
		  {
			  memcpy(&tx_buffer[tx_idx], &kP, sizeof(float));
			  tx_idx += sizeof(float);

			  memcpy(&tx_buffer[tx_idx], &kI, sizeof(float));
			  tx_idx += sizeof(float);

			  memcpy(&tx_buffer[tx_idx], &kD, sizeof(float));
			  tx_idx += sizeof(float);

			  memcpy(&tx_buffer[tx_idx], &Ts, sizeof(float));
			  tx_idx += sizeof(float);
		  }

		  // --- BLOQUE 3: Scheduling ---
		  if (flag_send_scheduling)
		  {
			  Log_Event(2); // Evento 2: Inicio transmisión UART DMA

			  uint8_t count = LOG_BUFFER_SIZE;
			  tx_buffer[tx_idx++] = count;

			  for (uint8_t i = 0; i < count; i++) {
				  memcpy(&tx_buffer[tx_idx], &log_buffer[i].timestamp, sizeof(uint32_t));
				  tx_idx += sizeof(uint32_t);

				  tx_buffer[tx_idx++] = log_buffer[i].event_id;
			  }
		  }

		  // --- Pie y CRC8 ---
		  uint8_t crc8 = Calculate_CRC8(tx_buffer, tx_idx);
		  tx_buffer[tx_idx++] = crc8;

		  // Disparo no bloqueante vía DMA (libera la CPU inmediatamente)
		  uart_tx_ready = 0;
		  HAL_UART_Transmit_DMA(&huart2, tx_buffer, tx_idx);
	  }

	  // Cadencia del ciclo principal de telemetría (ej. 50 ms)
	  HAL_Delay(50);
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Log_Event(1); // Registrar inicio del proceso de control PID

        // 1. Extraer lecturas del buffer DMA
        uint16_t sensor_val = adc_buffer[0]; // Canal 0
        uint16_t ref_val    = adc_buffer[1]; // Canal 1

        // 2. Ejecutar PID
        dac_val = PID_Compute(sensor_val, ref_val, kP, kI, kD, Ts);

        // 3. Escribir resultado en el DAC
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);
    }
}

// Callback cuando el DMA termina de transmitir la telemetría por UART
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        Log_Event(3); // Evento 3: Fin de transmisión UART vía DMA
        uart_tx_ready = 1; // Marcar UART libre para el siguiente envío
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
