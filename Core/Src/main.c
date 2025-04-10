/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
typedef enum {
    STATE_0,  // Afișare PRG, MOD, DAT, START
    STATE_1,  // Așteaptă START
    STATE_2,  // Configurare CDA - CLK OUT, ACK IN, DATA OUT
    STATE_3,  // Așteaptă DAT
    STATE_4,  // Verificare MOD
    STATE_5,  // QAM-Tx - CLK OUT, ACK IN, DATA OUT
    STATE_6,  // QAM-Rx - CLK IN, ACK OUT, DATA IN
} FSM_State_t;// enumerare care defineste un nou tip de date

FSM_State_t currentState = STATE_0;  // Inițializare în stare 0

typedef enum {
    MODE_QAM_TX,  // Transmitere (ARM → DSP)
    MODE_QAM_RX   // Receptie (DSP → ARM)
} OperationMode_t;

OperationMode_t currentMode = MODE_QAM_TX;  // Inițializare pe transmitere
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void ProcessFSM(void);
void Configure_GPIO_Mode(OperationMode_t mode);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_TIM_Base_Start_IT(&htim2);

  Configure_GPIO_Mode(MODE_QAM_TX);  // Inițializare inițială în Tx
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(GPIOA, LED_0_Pin|LED_1_Pin|LED_2_Pin|CDA_0_Pin
                          |CDA_1_Pin|CLK_Pin|ACK_Pin|RGB_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGB_2_Pin|RGB_3_Pin|LED_3_Pin|LED_4_Pin
                          |LED_5_Pin|DATA_0_Pin|DATA_1_Pin|DATA_2_Pin
                          |DATA_3_Pin|LED_6_Pin|LED_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Pin */
  GPIO_InitStruct.Pin = USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_0_Pin LED_1_Pin LED_2_Pin CDA_0_Pin
                           CDA_1_Pin CLK_Pin ACK_Pin RGB_1_Pin */
  GPIO_InitStruct.Pin = LED_0_Pin|LED_1_Pin|LED_2_Pin|CDA_0_Pin
                          |CDA_1_Pin|CLK_Pin|ACK_Pin|RGB_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_2_Pin RGB_3_Pin LED_3_Pin LED_4_Pin
                           LED_5_Pin DATA_0_Pin DATA_1_Pin DATA_2_Pin
                           DATA_3_Pin LED_6_Pin LED_7_Pin */
  GPIO_InitStruct.Pin = RGB_2_Pin|RGB_3_Pin|LED_3_Pin|LED_4_Pin
                          |LED_5_Pin|DATA_0_Pin|DATA_1_Pin|DATA_2_Pin
                          |DATA_3_Pin|LED_6_Pin|LED_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : START_Pin DAT_Pin PRG_Pin MOD_Pin */
  GPIO_InitStruct.Pin = START_Pin|DAT_Pin|PRG_Pin|MOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B_0_Pin B_1_Pin B_2_Pin B_3_Pin */
  GPIO_InitStruct.Pin = B_0_Pin|B_1_Pin|B_2_Pin|B_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



static uint8_t cda;
void ARM_WR_CDA(void)
{
    static int subState = 10;

    switch (subState)
    {
        case 10:
            // Calculare CDA = 2 * PRG + MOD
            uint8_t prg = HAL_GPIO_ReadPin(GPIOA, PRG_PIN); // PRG
            uint8_t mod = HAL_GPIO_ReadPin(GPIOA, MOD_PIN); // MOD
            cda = 2 * prg + mod;

            subState = 11;
            break;

        case 11:
            HAL_GPIO_WritePin(GPIOA, CLK_PIN, GPIO_PIN_RESET); // CLK LOW
            // Scriem bitii CDA pe PA4 (bit0) și PA5 (bit1)
            HAL_GPIO_WritePin(GPIOA, CDA_0_PIN, (cda & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, CDA_0_PIN, (cda & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            subState = 12;
            break;

        case 12:
            HAL_GPIO_WritePin(GPIOA, CLK_PIN, GPIO_PIN_SET);   // CLK HIGH
            subState = 13;
            break;

        case 13:
            // Asteapta ca DSP-ul sa traga ACK pe 0 (LOW)
            if (HAL_GPIO_ReadPin(GPIOA, ACK_PIN) == GPIO_PIN_RESET)
            {
                subState = 14;
            }
            break;

        case 14:
            // Asteapta ca DSP-ul sa ridice ACK pe 1 (HIGH)
            if (HAL_GPIO_ReadPin(GPIOA, ACK_PIN) == GPIO_PIN_SET)
            {
                subState = 10;
                currentState = STATE_0; // Revenim la starea initiala
            }
            break;

    }
}

void ARM_WR_DATA(void)
{
    static int subState = 30;

    switch (subState)
    {
        case 30:
            uint8_t D0   = HAL_GPIO_ReadPin(GPIOB, B_0_Pin);
            uint8_t D1   = HAL_GPIO_ReadPin(GPIOB, B_1_Pin);
            uint8_t D2   = HAL_GPIO_ReadPin(GPIOB, B_2_Pin);
            uint8_t D3 = HAL_GPIO_ReadPin(GPIOB, B_3_Pin);

            HAL_GPIO_WritePin(GPIOB, DATA_0_Pin, D0);
            HAL_GPIO_WritePin(GPIOB, DATA_1_Pin, D1);
            HAL_GPIO_WritePin(GPIOB, DATA_2_Pin, D2);
            HAL_GPIO_WritePin(GPIOB, DATA_3_Pin, D3);

            // CLK = 0
            HAL_GPIO_WritePin(GPIOA, CLK_PIN, GPIO_PIN_RESET);
            subState = 31;
            break;

        case 31:
            // CLK = 1 (trimitem semnal de ceas)
            HAL_GPIO_WritePin(GPIOA, CLK_PIN, GPIO_PIN_SET);
            subState = 32;
            break;

        case 32:
            // Așteptăm ca ACK să devină 0 (de la DSP)
            if (HAL_GPIO_ReadPin(GPIOA, ACK_PIN) == GPIO_PIN_RESET)
                subState = 33;
            break;

        case 33:
            // Așteptăm ACK = 1 => confirmare finală
            if (HAL_GPIO_ReadPin(GPIOA, ACK_PIN) == GPIO_PIN_SET)
            {
                subState = 30;
                currentState = STATE_0; // revenim la starea inițială
            }
            break;
    }
}

void ARM_RD_DATA(void)
{
    static int subState = 60;

    switch (subState)
    {
        case 60:
            // Așteptăm CLK să fie LOW
            if (HAL_GPIO_ReadPin(GPIOA, CLK_Pin) == GPIO_PIN_RESET)
                subState = 61;
            break;

        case 61:
            // Așteptăm CLK să devină HIGH
            if (HAL_GPIO_ReadPin(GPIOA, CLK) == GPIO_PIN_SET)
                subState = 62;
            break;

        case 62:
        {
        	uint8_t received_data = (HAL_GPIO_ReadPin(GPIOB, DATA_0_Pin) << 3) |
        	                        (HAL_GPIO_ReadPin(GPIOB, DATA_1_Pin) << 2) |
        	                        (HAL_GPIO_ReadPin(GPIOB, DATA_2_Pin) << 1) |
        	                        (HAL_GPIO_ReadPin(GPIOB, DATA_3_Pin));


        	HAL_GPIO_WritePin(GPIOB, LED_4_PIN, received_data & 0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED0
        	HAL_GPIO_WritePin(GPIOB, LED_5_PIN, received_data & 0x02 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED1
        	HAL_GPIO_WritePin(GPIOB, LED_6_PIN, received_data & 0x04 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED2
        	HAL_GPIO_WritePin(GPIOB, LED_7_PIN, received_data & 0x08 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED3


            // ACK = 0
            HAL_GPIO_WritePin(GPIOA, ACK_PIN, GPIO_PIN_RESET);
            subState = 63;
        }

        case 63:
            // ACK = 1
            HAL_GPIO_WritePin(GPIOA, ACK_PIN, GPIO_PIN_SET);

            if (HAL_GPIO_ReadPin(GPIOA, ACK_PIN) == GPIO_PIN_SET)
            {
                subState = 60;
                currentState = STATE_0;
            }
            break;

        default:
            break;
    }
}


void ProcessFSM(void)// automat cu stari finite
{

	// Aprindere LED-uri externe în funcție de butoane apăsate
    uint8_t prg   = HAL_GPIO_ReadPin(GPIOA, PRG_PIN);
    uint8_t mod   = HAL_GPIO_ReadPin(GPIOA, MOD_PIN);
    uint8_t dat   = HAL_GPIO_ReadPin(GPIOA, DAT_PIN);
    uint8_t start = HAL_GPIO_ReadPin(GPIOA, START_PIN);

    HAL_GPIO_WritePin(GPIOA, LED_0_PIN,  prg   ? GPIO_PIN_SET : GPIO_PIN_RESET); // PRG extern
    HAL_GPIO_WritePin(GPIOA, LED_1_PIN,  mod   ? GPIO_PIN_SET : GPIO_PIN_RESET); // MOD extern
    HAL_GPIO_WritePin(GPIOA, LED_2_PIN,  dat   ? GPIO_PIN_SET : GPIO_PIN_RESET); // DAT extern
    HAL_GPIO_WritePin(GPIOB, LED_3_PIN, start ? GPIO_PIN_SET : GPIO_PIN_RESET); // START extern

    switch (currentState)
    {
        case STATE_0:
            // resetare data
            HAL_GPIO_WritePin(GPIOB, DATA_3_PIN, GPIO_PIN_RESET); // DATA3 OFF
            HAL_GPIO_WritePin(GPIOB, DATA_2_PIN, GPIO_PIN_RESET); // DATA2 OFF
            HAL_GPIO_WritePin(GPIOB, DATA_1_PIN, GPIO_PIN_RESET); // DATA1 OFF
            HAL_GPIO_WritePin(GPIOB, DATA_0_PIN, GPIO_PIN_RESET); // DATA0 OFF

            currentState = STATE_1;
            break;

        case STATE_1:
            if (HAL_GPIO_ReadPin(GPIOA, START_PIN) == GPIO_PIN_SET) // START apasat
            {
                currentState = STATE_2;
            }
            break;

        case STATE_2:
            // Configurare CLK OUT, ACK IN, DATA OUT
            Configure_GPIO_Mode(MODE_QAM_TX);

            ARM_WR_CDA();

            currentState = STATE_3;
            break;

        case STATE_3:
            if (HAL_GPIO_ReadPin(GPIOA, DAT_PIN) == GPIO_PIN_SET) // DAT apasat
            {
                currentState = STATE_4;
            }
            break;

        case STATE_4:
            if (HAL_GPIO_ReadPin(GPIOA, MOD_PIN) == GPIO_PIN_RESET) // MOD = 1 (Tx)
            {
                currentState = STATE_5;
            }
            else  // MOD = 0 (Rx)
            {
                currentState = STATE_6;
            }
            break;

        case STATE_5:
            // Setam directia GPIO pentru transmisie
            Configure_GPIO_Mode(MODE_QAM_TX);

            // Generam semnal de CLK pentru a semnala ca datele sunt gata
            HAL_GPIO_WritePin(GPIOA, CLK_PIN, GPIO_PIN_SET); // CLK HIGH
            HAL_Delay(1); // Mic delay pentru stabilitate

            HAL_GPIO_WritePin(GPIOA, CLK_PIN, GPIO_PIN_RESET); // CLK LOW
            HAL_Delay(1); // Mic delay

            // Așteptăm ACK de la DSP (PA7 trebuie să devină HIGH)
            while (HAL_GPIO_ReadPin(GPIOA, ACK_PIN) == GPIO_PIN_RESET)
            {
                // Așteaptă
            }

            // (Opțional) Așteptăm să revină ACK la 0 (front de coborâre)
            while (HAL_GPIO_ReadPin(GPIOA, ACK_PIN) == GPIO_PIN_SET)
            {
                // Așteaptă ACK să fie eliberat
            }

            // Aprindere LED-uri corespunzătoare datelor transmise/ simbolului QAM transmis
            uint8_t d0 = HAL_GPIO_ReadPin(GPIOB, DATA_0_PIN); // DATA0
            uint8_t d1 = HAL_GPIO_ReadPin(GPIOB, DATA_1_PIN); // DATA1
            uint8_t d2 = HAL_GPIO_ReadPin(GPIOB, DATA_2_PIN); // DATA2
            uint8_t d3 = HAL_GPIO_ReadPin(GPIOB, DATA_3_PIN); // DATA3


            HAL_GPIO_WritePin(GPIOB, LED_4_PIN, d0 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED0
            HAL_GPIO_WritePin(GPIOB, LED_5_PIN, d1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED1
            HAL_GPIO_WritePin(GPIOB, LED_6_PIN, d2 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED2
            HAL_GPIO_WritePin(GPIOB, LED_7_PIN, d3 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED3

            ARM_WR_DATA();
            currentState = STATE_0; // Revenim la stare inițială
            break;


        case STATE_6:
            // Setam directia GPIO pentru receptie
            Configure_GPIO_Mode(MODE_QAM_RX);

            // Asteptam semnal de ceas de la DSP
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) // CLK HIGH detectat
            {
                // Citim datele de la DSP
                uint8_t received_data = (HAL_GPIO_ReadPin(GPIOB, DATA_3_PIN) << 3) |
                                        (HAL_GPIO_ReadPin(GPIOB, DATA_2_PIN) << 2) |
                                        (HAL_GPIO_ReadPin(GPIOB, DATA_1_PIN) << 1) |
                                        (HAL_GPIO_ReadPin(GPIOB, DATA_0_PIN));


                //extragere biti din byte
                uint8_t d0 = received_data & 0x01;
                uint8_t d1 = (received_data >> 1) & 0x01;
                uint8_t d2 = (received_data >> 2) & 0x01;
                uint8_t d3 = (received_data >> 3) & 0x01;

                HAL_GPIO_WritePin(GPIOB, LED_4_PIN, d0 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED0
                HAL_GPIO_WritePin(GPIOB, LED_5_PIN, d1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED1
                HAL_GPIO_WritePin(GPIOB, LED_6_PIN, d2 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED2
                HAL_GPIO_WritePin(GPIOB, LED_7_PIN, d3 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED3

                // Trimitem semnal de confirmare (ACK)
                HAL_GPIO_WritePin(GPIOA, ACK_PIN, GPIO_PIN_SET); // ACK HIGH
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOA, ACK_PIN, GPIO_PIN_RESET); // ACK LOW

                ARM_RD_DATA();

                currentState = STATE_0;
            }
            break;

    }
}

void Configure_GPIO_Mode(OperationMode_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DATA_0_PIN | DATA_1_PIN | DATA_2_PIN | DATA_3_PIN;
    GPIO_InitStruct.Mode = (mode == MODE_QAM_TX) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CLK_PIN;
    GPIO_InitStruct.Mode = (mode == MODE_QAM_TX) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ACK_PIN;
    GPIO_InitStruct.Mode = (mode == MODE_QAM_TX) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        ProcessFSM();
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
