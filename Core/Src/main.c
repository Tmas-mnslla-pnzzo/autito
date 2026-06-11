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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "sg90.h"
#include "hcsr04.h"
#include "soft_i2c.h"
#include "ssd1306.h"
#include "ESP01.h"
#include "protocolo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t valores_IR[3];
volatile uint16_t tick_100ms = 0;
volatile uint8_t flag_loop = 0;
volatile uint32_t tick_servo = 0;
uint8_t display_mode = 0;
volatile uint8_t flag_2seg = 0;
volatile uint32_t now_us = 0;
volatile float distancia = 0;
volatile uint16_t pwm_actual = 500;
//uint16_t adc_buf[3];
char pc_ip[20] = "192.168.1.10";
//192.168.1.10
uint8_t uart1_rx_byte;
uint8_t esp_rx_byte;
HCSR04_t sonar;
SG90_t servo;
SSD1306_t display;
_sESP01Handle hESP01;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void onESP01StateChange(_eESP01STATUS state);
void CmdParser(uint8_t cmd, uint8_t *payload, uint8_t n);
void servo_set_pin(uint8_t state);
void trigger_set(uint8_t state);
uint8_t echo_get(void);
void sonar_result(float dist_cm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ESP01_DebugCallback(const char *str)
{
    // Canal 1: crudo por UART1
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 10);

    // Canal 2: empaquetado como TEL_LOG para que la HMI lo muestre bonito
    uint8_t len = strlen(str);
    Encode(0x33, (uint8_t*)str, len);
    uint8_t send_buf[64];
    uint8_t send_len = 0;
    while (tx.rBuf.ir != tx.rBuf.iw) {
        send_buf[send_len++] = tx.rBuf.buf[tx.rBuf.ir++];
        tx.rBuf.ir &= (tx.rBuf.size - 1);
    }
    if (send_len > 0)
        HAL_UART_Transmit(&huart1, send_buf, send_len, 100);
}
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

void ESP01_DoCHPD(uint8_t value)
{
    // CH_PD fijo a 3.3V, no necesita control
}

int ESP01_WriteUSARTByte(uint8_t value)
{
    if (HAL_UART_Transmit(&huart3, &value, 1, 10) == HAL_OK)
        return 1;
    return 0;
}

void ESP01_WriteByteToBufRX(uint8_t value)
{
    rx.rBuf.buf[rx.rBuf.iw++] = value;
    rx.rBuf.iw &= (rx.rBuf.size - 1);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Protocolo_Init();
  Protocolo_SetCmdParser(CmdParser);
  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  HAL_ADCEx_Calibration_Start(&hadc1);
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 3);
  HAL_TIM_Base_Start_IT(&htim2);           // tick cada 100µs
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // EN1 - PB0
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // EN2 - PB1
  SG90_Init(&servo, servo_set_pin);
  HCSR04_Init(&sonar, trigger_set, echo_get, sonar_result);

  SSD1306_Init(&display, soft_i2c_write, SSD1306_ADDR);
  SSD1306_Clear(&display);
  SSD1306_DrawText(&display, 0, 0, "UNER-MECATRONICA", 1);
  SSD1306_Update(&display);

  uint8_t test[] = "AT\r\n";
  HAL_UART_Transmit(&huart3, test, 4, 100);

  hESP01.DoCHPD          = ESP01_DoCHPD;
  hESP01.WriteUSARTByte  = ESP01_WriteUSARTByte;
  hESP01.WriteByteToBufRX = ESP01_WriteByteToBufRX;

  HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
  ESP01_Init(&hESP01);
  ESP01_AttachDebugStr(ESP01_DebugCallback);
  ESP01_AttachChangeState(onESP01StateChange);
  ESP01_SetWIFI("InternetPlus_2e7438", "wland18bc7");
  //172.23.231.216
  //192.168.1.10
  //el algoritmo debe ser lo mas inmune posible a la luz, interpolacion cuadratica.el que esta mas oroximo es el que teinemas valor
  //punto auxiliar o ficticio, hace una parabola y el vertice es donde esta la linea, usar todo el rango dinamico, control tener en cunata la panza diodo curva aplicando PID, dibujar cuadraditos para el control
  //
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ESP01_Task();
	  Decode();

	  if (flag_2seg) {
	      flag_2seg = 0;
	      if (display_mode == 0) {
	          display_mode = 1;  // después de 2 seg pasamos a telemetría
	      }
	  }

      if (flag_loop)
      {
          flag_loop = 0;
          uint32_t t = now_us;
          HCSR04_Trigger(&sonar, t);

          // Leer IR
          ADC_ChannelConfTypeDef sConfig = {0};
          sConfig.Rank = ADC_REGULAR_RANK_1;
          sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

          sConfig.Channel = ADC_CHANNEL_0;
          HAL_ADC_ConfigChannel(&hadc1, &sConfig);
          HAL_ADC_Start(&hadc1);
          HAL_ADC_PollForConversion(&hadc1, 10);
          uint32_t adc_izq = HAL_ADC_GetValue(&hadc1);
          HAL_ADC_Stop(&hadc1);

          sConfig.Channel = ADC_CHANNEL_1;
          HAL_ADC_ConfigChannel(&hadc1, &sConfig);
          HAL_ADC_Start(&hadc1);
          HAL_ADC_PollForConversion(&hadc1, 10);
          uint32_t adc_cen = HAL_ADC_GetValue(&hadc1);
          HAL_ADC_Stop(&hadc1);

          sConfig.Channel = ADC_CHANNEL_2;
          HAL_ADC_ConfigChannel(&hadc1, &sConfig);
          HAL_ADC_Start(&hadc1);
          HAL_ADC_PollForConversion(&hadc1, 10);
          uint32_t adc_der = HAL_ADC_GetValue(&hadc1);
          HAL_ADC_Stop(&hadc1);

          uint8_t ir_izq = (adc_izq > 1900) ? 1 : 0;
          uint8_t ir_cen = (adc_cen > 950)  ? 1 : 0;
          uint8_t ir_der = (adc_der > 950)  ? 1 : 0;

          uint16_t dist_int = (distancia < 0) ? 0xFFFF : (uint16_t)distancia;

          SSD1306_Clear(&display);
              if (display_mode == 0) {
                  SSD1306_DrawText(&display, 0, 0, "UNER-MECATRONICA", 1);
              } else {
                  char buf[20];
                  sprintf(buf, "IR:%d%d%d", ir_izq, ir_cen, ir_der);
                  SSD1306_DrawText(&display, 0, 0, buf, 1);
                  sprintf(buf, "Dist: %d cm", dist_int == 0xFFFF ? 9999 : dist_int);
                  SSD1306_DrawText(&display, 0, 16, buf, 1);
                  sprintf(buf, "PWM: %d", pwm_actual);
                  SSD1306_DrawText(&display, 0, 32, buf, 1);
              }
              SSD1306_Update(&display);

          uint8_t payload[5];
                    payload[0] = ir_izq;
                    payload[1] = ir_cen;
                    payload[2] = ir_der;
                    payload[3] = (uint8_t)(dist_int >> 8);   // Byte ALTO
                    payload[4] = (uint8_t)(dist_int & 0xFF); // Byte BAJO

                    // 2. Delegamos el empaquetado y el cálculo matemático del CKS al protocolo
                    Encode(0x30, payload, 5);

                    // 3. Extraemos los bytes del búfer circular de transmisión (tx)
                    uint8_t send_buf[32];
                    uint8_t send_len = 0;

                    // Vaciamos el búfer circular hacia un arreglo lineal para DMA/Polling
                    // (Asumiendo que 'tx' está accesible vía protocolo.h)
                    while (tx.rBuf.ir != tx.rBuf.iw)
                    {
                        send_buf[send_len++] = tx.rBuf.buf[tx.rBuf.ir++];
                        // El enmascaramiento bit a bit asegura la circularidad formal (potencia de 2)
                        tx.rBuf.ir &= (tx.rBuf.size - 1);
                    }

                    // 4. Transmitimos el datagrama completo a las interfaces físicas
                    if (send_len > 0) {
                        HAL_UART_Transmit(&huart1, send_buf, send_len, 100);
                        if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED) {
                            if (ESP01_Send(send_buf, 0, send_len, send_len) == ESP01_SEND_READY) {
                                // ok
                            }
                        }
                    }

      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEARBEAT_GPIO_Port, HEARBEAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIGGER_Pin|IN2_Pin|IN1_Pin|SCL_Pin
                          |SDA_Pin|IN4_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HEARBEAT_Pin */
  GPIO_InitStruct.Pin = HEARBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEARBEAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW0_Pin SW1_Pin SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW0_Pin|SW1_Pin|SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIGGER_Pin IN2_Pin IN1_Pin SCL_Pin
                           SDA_Pin IN4_Pin IN3_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin|IN2_Pin|IN1_Pin|SCL_Pin
                          |SDA_Pin|IN4_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SERVO_Pin */
  GPIO_InitStruct.Pin = SERVO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERVO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
    }
}
void CmdParser(uint8_t cmd, uint8_t *payload, uint8_t n)
{
    switch (cmd)
    {
    	case 0x21:
    	    char *ssid = (char*)payload;
    	    char *pass = ssid + strlen(ssid) + 1;
    	    ESP01_SetWIFI(ssid, pass);
    	    break;
        case 0x10: // CMD_MOVE_FORWARD
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_actual);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_actual);
            break;

        case 0x11: // CMD_MOVE_BACKWARD
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_actual);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_actual);
            break;

        case 0x12: // CMD_TURN_LEFT
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_actual);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_actual);
            break;

        case 0x13: // CMD_TURN_RIGHT
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_actual);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_actual);
            break;

        case 0x14: // CMD_STOP
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
         	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_RESET);
        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        	break;
        case 0x15: // CMD_SET_PWM
            if (n >= 1) {
            	pwm_actual = (payload[0] * 999) / 100;
            }
            break;

        case 0x16: // CMD_SET_SERVO
            if (n >= 1) SG90_SetAngle(&servo, payload[0]);
            break;

        default:
            break;
    }
}
void onESP01StateChange(_eESP01STATUS state)
{
    if (state == ESP01_WIFI_CONNECTED)
    {
        ESP01_StartUDP("192.168.1.10", 5000, 5000);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Meter byte en buffer del protocolo
        rx.rBuf.buf[rx.rBuf.iw++] = uart1_rx_byte;
        rx.rBuf.iw &= (rx.rBuf.size - 1);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    if (huart->Instance == USART3)
    {
        ESP01_WriteRX(esp_rx_byte);
        HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
    }
}
void sonar_result(float dist_cm)
{
    distancia = dist_cm;
}
void trigger_set(uint8_t state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
uint8_t echo_get(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) ? 1 : 0;
}
void servo_set_pin(uint8_t state)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        now_us += 100;
        SG90_Tick(&servo, 100);
        HCSR04_Tick(&sonar, now_us);

        // Timer 10ms para ESP01
        static uint16_t esp_tick = 0;
        esp_tick++;
        if (esp_tick >= 100)  // 100 x 100µs = 10ms
        {
            esp_tick = 0;
            ESP01_Timeout10ms();  // ← ESTO FALTABA
        }

        tick_100ms++;
        if (tick_100ms >= 5000)
        {
            tick_100ms = 0;
            flag_loop = 1;
        }

        tick_servo++;
        if (tick_servo >= 20000)
        {
            tick_servo = 0;
            flag_2seg = 1;
        }
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
