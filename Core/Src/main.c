/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MD22_I2C_ADDRESS (0xB0)
#define SPEED_OF_SOUND 343.0f
#define BTBUFFER_SIZE 50
#define ACCELERATION 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile int8_t rightSpeed = 0;
volatile int8_t leftSpeed = 0;

volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint8_t echo_received = 0;

volatile uint8_t uart_rx_buffer[3]; // Buffer para almacenar los datos recibidos
volatile uint8_t uart_rx_buffer_index = 0;
volatile uint8_t validMove = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

volatile void debugMsg(char* cadena)
{
  	 HAL_UART_Transmit(&huart1, (uint8_t*) cadena, strlen(cadena), 1000);
}

void MD22_Init(void) {
    uint8_t data[2];

    // Configurar el modo de control en "Modo 0"
    data[0] = 0;  // Registro 0: Modo de control
    data[1] = 0;  // Modo 0: Control directo de velocidades


    if (HAL_I2C_Master_Transmit(&hi2c1, MD22_I2C_ADDRESS, data, 2, 100) != HAL_OK) {
        debugMsg("MAL");
    }else{
    	debugMsg("BIEN");
    }
}

volatile void MD22_SetMotors(uint8_t speed_motor1, uint8_t speed_motor2) {
    uint8_t data[3];

    debugMsg("Intentando encender el motor \r");
    // Establecer velocidades en los registros 1 y 2
    data[0] = 1;           // Registro 1: Velocidad Motor 1
    data[1] = speed_motor1; // Velocidad Motor 1 (0-255)
    data[2] = speed_motor2; // Velocidad Motor 2 (0-255)

    if (HAL_I2C_Master_Transmit(&hi2c1, MD22_I2C_ADDRESS, data, 3, 100) != HAL_OK) {
        debugMsg("MAL\r");
    }else{
    	debugMsg("BIEN\r");
    }
}

int measure_distance(void) {
	debugMsg("Midiendo Distancia \r");

    uint32_t elapsed_time;
    float distance;

    // Generar pulso TRIGGER
    debugMsg("Disparando pulso \r");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  // TRIGGER ON
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    while (__HAL_TIM_GET_COUNTER(&htim2) < 10);  // Espera 10 µs
    HAL_TIM_Base_Stop(&htim2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);  // TRIGGER OFF

    uint32_t timeout = 20000; // Tiempo máximo de espera en microsegundos
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
        if (timeout == 0) {
            debugMsg("Timeout: No se recibió pulso ECHO. Distancia máxima asumida (500 m).\r");
            return 500.0f; // Distancia máxima asumida en metros
        }
        timeout--;
    }

    //debugMsg("Midiendo pulso \r");
    // Iniciar temporizador TIM2
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);

    // Esperar a que ECHO se desactive
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET);
    //debugMsg("pulso medido \r");

    // Detener temporizador y obtener tiempo transcurrido
    HAL_TIM_Base_Stop(&htim2);
    elapsed_time = __HAL_TIM_GET_COUNTER(&htim2);

    // Calcular distancia en metros
    char delta[50];
    sprintf(delta, "delta: %d segundos\r", (int) elapsed_time);
    debugMsg(delta);

    distance = ((elapsed_time / 1000.0f) * 343) / 2.0f;

    char buffer[50];
    sprintf(buffer, "Distancia medida: %d mm\r", (int) round(distance));
    debugMsg(buffer);

    return (int) round(distance);
}

char* uint8_to_string( volatile uint8_t* uint8_array, size_t length) {
    if (uint8_array == NULL || length == 0) return NULL;

    char* result = (char*)malloc(length + 1);
    if (result == NULL) {
        debugMsg("Error al asignar memoria");
        return NULL;
    }

    memcpy(result, uint8_array, length);
    result[length] = '\0';
    return result;
}

uint8_t getRightSpeed(uint8_t speed)
{
	if (speed == 0 ) return 127;
	else if (speed > 0 ) return speed + 127;
	else return 127 - speed;
}

uint8_t getLefttSpeed(uint8_t speed)
{
	if (speed == 0 ) return 127;
	else if (speed > 0 ) return 127 - speed;
	else return speed + 127;
}

int8_t clampSpeed(int val) {
    if (val > 127) return 127;
    if (val < -127) return -127;
    return (int8_t)val;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer)); // Limpia todo el buffer
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_buffer_index], 1);
  /* USER CODE BEGIN 2 */
  rightSpeed = 0;
  leftSpeed = 0;
  uint8_t isChoque = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 if ( measure_distance() <  150)
	 {
		 debugMsg("SE CHOCA!!\r");
		 rightSpeed = -127;
		 leftSpeed = -127;
		 MD22_SetMotors(getRightSpeed(rightSpeed), getLefttSpeed(leftSpeed));
		 isChoque = 1;
		 continue;
	 }else{
		 if (isChoque)
		 {
			 rightSpeed = 0;
			 leftSpeed = 0;
		 }
		 	 isChoque = 0;
	 }
	 char cadena [40];
	 sprintf(cadena, "Marcador %d: %c \r", uart_rx_buffer_index, (char) (uart_rx_buffer[uart_rx_buffer_index]));
	 debugMsg(cadena);

	 switch (uart_rx_buffer[validMove]){
		 case 'w':
			 rightSpeed = clampSpeed(rightSpeed + ACCELERATION);
			 leftSpeed = clampSpeed(leftSpeed + ACCELERATION);
			 break;
		 case 'a':
			 leftSpeed = 127;
			 rightSpeed = 0;
			 break;
		 case 's':
			 rightSpeed = clampSpeed(rightSpeed - ACCELERATION);
			 leftSpeed = clampSpeed(leftSpeed - ACCELERATION);
			 break;
		 case 'd':
			 leftSpeed = 0;
			 rightSpeed = 127;
			 break;
		 case 'm': // Parar las dos ruedas
			 rightSpeed = 0;
			 leftSpeed = 0;
			 break;
		 case 'x': // Parar las dos ruedas
			 rightSpeed = -127;
			 leftSpeed = 0;
			 break;
		 case 'b': // Parar las dos ruedas
			 rightSpeed = 0;
			 leftSpeed = -127;
			 break;
		 case 'o': // Parar las dos ruedas
			 rightSpeed = 127;
			 leftSpeed = -127;
			 break;
		 case 'k':
			 rightSpeed = -127;
			 leftSpeed = 127;
	 }
	 MD22_SetMotors(getRightSpeed(rightSpeed), getLefttSpeed(leftSpeed));
	 //debugMsg((char *)uart_rx_buffer);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
