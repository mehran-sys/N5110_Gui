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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "pcd8544_driver.h"
#include "pcd8544_gui.h"
#include "queue.h"
#include "stdio.h"
#include "math.h"
#include "stdbool.h"
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

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
MenuItems_e currentMenu;

TaskHandle_t mainMenu_handle;
TaskHandle_t mcuTempMenu_handle;
TaskHandle_t clockMenu_handle;
TaskHandle_t backlightMenu_handle;
TaskHandle_t contrastMenu_handle;
TaskHandle_t helpMenu_handle;
TaskHandle_t rebootMenu_handle;
TaskHandle_t timerMenu_handle;
TaskHandle_t handleButtonPressed_handle;

TimerHandle_t iwdgReloadTimer_handle;

QueueHandle_t pressedButtonQueue;

const char* MenuItemNames[MENU_ITEMS] = {" Clock"," Timer"," BackLight"," Contrast"," MCU Temp"," Help"," Reboot"};
const char* weekDays[7] = {"MONDAY","TUESDAY","WEDNESDAY","THURSDAY","FRIDAY","SATURDAY","SUNDAY"};

uint8_t currentPage = 0;
uint8_t currentMenuIndex = 0;
uint8_t menuIndex = 0;

uint8_t contrastValue;
int16_t backLightValue = 999;

uint8_t		timerMenuSeconds = 0;
uint8_t 	timerMenuMinutes = 0;
volatile uint8_t   timerMenuCentiSeconds = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void showMainMenu(void *pvParameters);
void showMcuTempMenu(void *pvParameters);
void showClockMenu(void *pvParameters);
void showBackLightMenu(void *pvParameters);
void showContrastMenu(void *pvParameters);
void showHelpMenu(void *pvParameters);
void showRebootMenu(void *pvParameters);
void showTimerMenu(void *pvParameters);

void handleButtonPressed(void *pvParameters);
void refreshIwdg();

void returnToMainMenu();
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
/*-------------------------Show Start Prompt---------------------------*/
  PCD8544_init(PCD8544_DEFAULT_CONTRAST);
  PCD8544_setFont(1);
  PCD8544_clearScreen();
  PCD8544_printStringAlign("LOADING...", 10, ALIGNMETN_CENTER);
  PCD8544_refreshScreen();
  HAL_Delay(500);
  PCD8544_clearScreen();
  PCD8544_refreshScreen();

  /*---------------------Set Contrast And BackLight--------------------*/
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, DEFAULT_BACKLIGHT_VALUE);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /*-------------------------Create Basic Tasks------------------------*/

  xTaskCreate(showMainMenu,
							"Main Menu",
							128,
							NULL,
							1,
							&mainMenu_handle);

  xTaskCreate(handleButtonPressed,
  						"Handle Key",
							128,
							NULL,
							1,
							&handleButtonPressed_handle);

  /*---------------------------Create Mics--------------------------*/
  pressedButtonQueue = xQueueCreate(5,sizeof(Buttons_e));

  iwdgReloadTimer_handle = xTimerCreate("IWDG Handle Timer",
  																			IWDG_REFRESH_TIME,
																				pdTRUE,
																				0,
																				refreshIwdg);
  /*-------------------------Start The Scheduler----------------------*/
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Init scheduler */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */


  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
 if (HAL_GPIO_ReadPin(CENTER_BUTTON_INPUT_GPIO_Port, CENTER_BUTTON_INPUT_Pin) == 1){
	RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

	sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
 }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_BUTTON_INPUT_Pin CENTER_BUTTON_INPUT_Pin RIGHT_BUTTON_INPUT_Pin */
  GPIO_InitStruct.Pin = LEFT_BUTTON_INPUT_Pin|CENTER_BUTTON_INPUT_Pin|RIGHT_BUTTON_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*----------------------Tasks------------------------*/

void handleButtonPressed(void *pvParameters){
	Buttons_e pressedButton;

	for (;;){
		if(HAL_GPIO_ReadPin(RIGHT_BUTTON_INPUT_GPIO_Port, RIGHT_BUTTON_INPUT_Pin) == 1 ||
			HAL_GPIO_ReadPin(LEFT_BUTTON_INPUT_GPIO_Port, LEFT_BUTTON_INPUT_Pin) == 1 ||
			HAL_GPIO_ReadPin(CENTER_BUTTON_INPUT_GPIO_Port, CENTER_BUTTON_INPUT_Pin) == 1){

				if (HAL_GPIO_ReadPin(RIGHT_BUTTON_INPUT_GPIO_Port, RIGHT_BUTTON_INPUT_Pin) == 1){
					pressedButton = RIGHT_BUTTON;
				}
				else if (HAL_GPIO_ReadPin(LEFT_BUTTON_INPUT_GPIO_Port, LEFT_BUTTON_INPUT_Pin) == 1){
					pressedButton  = LEFT_BUTTON;
				}
				else if (HAL_GPIO_ReadPin(CENTER_BUTTON_INPUT_GPIO_Port, CENTER_BUTTON_INPUT_Pin) == 1){
					pressedButton = CENETR_BUTTON;
				}
				xQueueSend(pressedButtonQueue,&pressedButton,0);
				vTaskDelay(BUTTON_DELAY_TIME);
		}
		HAL_IWDG_Refresh(&hiwdg);
	}
}

void showMainMenu(void *pvParameters){
	Buttons_e pressedButton;

	for(;;){

		/*------------------------Draw Menu Items-----------------------*/
		PCD8544_GUI_displayMenuName("MAIN MENU");

		menuIndex = currentPage * 3 + currentMenuIndex;
		PCD8544_GUI_drawSideBar(80, 16, 26, menuIndex, MENU_ITEMS);

		uint8_t itemsInPage;
		if (currentPage < PAGES - 1){
			itemsInPage = ITEMS_PER_PAGE;
		}
		else {
			itemsInPage = MENU_ITEMS % 3;
		}

		for (uint8_t i = 0 ; i < itemsInPage ; i++){
			if (i%3 != currentMenuIndex){
				PCD8544_printStringAlign(MenuItemNames[(currentPage*3)+i],((11*i)+17), ALIGNMETN_LEFT);
			}
			else if (i == currentMenuIndex){
				PCD8544_setTextInvert(true);
				char s[14];
				strcpy(s,">");
				strcat(s,MenuItemNames[(currentPage*3)+i]);
				PCD8544_printStringAlign(s,((11*i)+17), ALIGNMETN_LEFT);
				PCD8544_setTextInvert(false);
			}
		}

		PCD8544_refreshScreen();

		/*-----------------------Handle button events--------------------*/
		if (xQueueReceive(pressedButtonQueue, &pressedButton, HAL_MAX_DELAY) == pdPASS){
			if (pressedButton == RIGHT_BUTTON){
				currentMenuIndex++;

				if (currentMenuIndex > itemsInPage - 1){
					currentPage++;
					currentMenuIndex = 0;
					if (currentPage > PAGES - 1){
						currentPage = 0;
					}
				}
			}
			else if (pressedButton == LEFT_BUTTON){
				if (currentMenuIndex > 0){
					currentMenuIndex--;
				}
				else if (currentMenuIndex <= 0){
					if (currentPage > 0){
						currentPage--;
					}
					else {
						currentPage = PAGES - 1;
					}

					if (currentPage < PAGES - 1){
						itemsInPage = ITEMS_PER_PAGE;
					}
					else {
						itemsInPage = MENU_ITEMS % ITEMS_PER_PAGE;
					}

					currentMenuIndex = itemsInPage - 1;
				}
			}
			else if (pressedButton == CENETR_BUTTON){
				currentMenu = (ITEMS_PER_PAGE * currentPage) + currentMenuIndex;
				/*-----------------------Execute Menus---------------------*/
				if (currentMenu == CLOCK){
					xTaskCreate(showClockMenu,
											"Clock Menu",
											128,
											NULL,
											1,
											&clockMenu_handle);
				}
				else if (currentMenu == TIMER){
					xTaskCreate(showTimerMenu,
											"Timer Menu",
											128,
											NULL,
											1,
											&timerMenu_handle);
				}
				else if (currentMenu == BACKLIGHT){
					xTaskCreate(showBackLightMenu,
											"BackLight Menu",
											128,
											NULL,
											1,
											&backlightMenu_handle);
				}
				else if (currentMenu == CONTRAST){
					xTaskCreate(showContrastMenu,
											"Contrast Menu",
											128,
											NULL,
											1,
											&timerMenu_handle);
				}
				else if (currentMenu == MCU_TEMP){
					xTaskCreate(showMcuTempMenu,
											"MCU Temp Menu",
											128,
											NULL,
											1,
											&mcuTempMenu_handle);
				}
				else if (currentMenu == HELP){
					xTaskCreate(showHelpMenu,
											"Help Menu",
											128,
											NULL,
											1,
											&helpMenu_handle);
				}
				else if (currentMenu == REBOOT){
					xTaskCreate(showRebootMenu,
											"Reboot Menu",
											128,
											NULL,
											1,
											&rebootMenu_handle);
				}

				PCD8544_clearScreen();
				PCD8544_refreshScreen();
				vTaskSuspend(NULL);
				HAL_Delay(10);
			}
		}

		PCD8544_clearScreen();
	}
}

void showMcuTempMenu(void *pvParameters){
	Buttons_e pressedButton;
	uint32_t  adcValue;
	int       temp;
	char      tempBuffer[10];

	for(;;){

		/*---------------------Convert Temperature----------------------*/
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, MENU_REFRESH_TIME);
		adcValue = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		temp = (1500-adcValue)/4;

		/*------------------------Draw Menu Items-----------------------*/
		PCD8544_GUI_displayMenuName("MCU TEMP");

		sprintf(tempBuffer, "%d", temp);
		PCD8544_setFont(6);
		PCD8544_printStringAlign(tempBuffer, 20, ALIGNMETN_CENTER);
		PCD8544_setFont(1);

		PCD8544_refreshScreen();

		/*-----------------------Handle button events--------------------*/
		if (xQueueReceive(pressedButtonQueue, &pressedButton, MENU_REFRESH_TIME) == pdPASS){
			if (pressedButton == RIGHT_BUTTON){

			}
			else if (pressedButton == LEFT_BUTTON){

			}
			else if (pressedButton == CENETR_BUTTON){
				returnToMainMenu();
			}
		}

		PCD8544_clearScreen();
	}
}

void showClockMenu(void *pvParameters){
	Buttons_e pressedButton;
	RTC_TimeTypeDef cTime;
	char            timeBuffer[10];
	uint8_t 				editCounter = 0;
	uint8_t					hour;
	uint8_t					minute;
	uint8_t					second;

	for (;;){
		HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BIN);

		PCD8544_GUI_displayMenuName("CLOCK");

		if (editCounter == 0){
			sprintf(timeBuffer, "%02d:%02d:%02d", cTime.Hours, cTime.Minutes, cTime.Seconds);
			PCD8544_setFont(4);
			PCD8544_printStringAlign(timeBuffer, 25, ALIGNMETN_CENTER);
			PCD8544_setFont(1);
		}
		else {
			if (editCounter == 1){
				char tempBuffer[3];
				sprintf(timeBuffer, "  :%02d:%02d", minute, second);
				PCD8544_setFont(4);
				PCD8544_printStringAlign(timeBuffer, 25, ALIGNMETN_CENTER);
				sprintf(tempBuffer, "%02d", hour);
				PCD8544_setTextInvert(true);
				PCD8544_printString(tempBuffer, 6, 25);
				PCD8544_setTextInvert(false);
				PCD8544_setFont(1);
			}
			else if (editCounter == 2){
				char tempBuffer[3];
				sprintf(timeBuffer, "%02d:  :%02d", hour, second);
				PCD8544_setFont(4);
				PCD8544_printStringAlign(timeBuffer, 25, ALIGNMETN_CENTER);
				sprintf(tempBuffer, "%02d", minute);
				PCD8544_setTextInvert(true);
				PCD8544_printString(tempBuffer, 33, 25);
				PCD8544_setTextInvert(false);
				PCD8544_setFont(1);
			}
			else if (editCounter == 3){
				char tempBuffer[3];
				sprintf(timeBuffer, "%02d:%02d:  ", hour, minute);
				PCD8544_setFont(4);
				PCD8544_printStringAlign(timeBuffer, 25, ALIGNMETN_CENTER);
				sprintf(tempBuffer, "%02d", second);
				PCD8544_setTextInvert(true);
				PCD8544_printString(tempBuffer, 60, 25);
				PCD8544_setTextInvert(false);
				PCD8544_setFont(1);
			}
		}

		PCD8544_refreshScreen();

		/*-----------------------Handle button events--------------------*/
		if (xQueueReceive(pressedButtonQueue, &pressedButton, MENU_REFRESH_TIME) == pdPASS){
			if (pressedButton == RIGHT_BUTTON){

				if (editCounter == 1){
					hour++;
					if (hour > 23){
						hour = 0;
					}
				}
				else if (editCounter == 2){
					minute++;
					if (minute > 59){
						minute = 0;
					}
				}
				else if (editCounter == 3){
					second++;
					if (second > 59){
						second = 0;
					}
				}
			}
			else if (pressedButton == LEFT_BUTTON){
				if (editCounter == 0){
					hour = cTime.Hours;
					minute = cTime.Minutes;
					second = cTime.Seconds;
				}

				if (editCounter == 1){
					cTime.Hours = hour;
					HAL_RTC_SetTime(&hrtc, &cTime, RTC_FORMAT_BIN);
				}
				else if (editCounter == 2){
					cTime.Minutes = minute;
					HAL_RTC_SetTime(&hrtc, &cTime, RTC_FORMAT_BIN);
				}
				else if (editCounter == 3){
					cTime.Seconds = second;
					HAL_RTC_SetTime(&hrtc, &cTime, RTC_FORMAT_BIN);
				}

				editCounter++;
				if (editCounter > 3){
					editCounter = 0;
				}
			}
			else if (pressedButton == CENETR_BUTTON){
				returnToMainMenu();
			}
		}

		PCD8544_clearScreen();
	}
}

void showBackLightMenu(void *pvParameters){
	Buttons_e pressedButton;

	for (;;){
		/*------------------------Draw Menu Items-----------------------*/
		PCD8544_GUI_displayMenuName("BACKLIGHT");


		PCD8544_GUI_drawProgressBar(4, 17, 76, 25, (ceil(backLightValue+1)/10));

		PCD8544_refreshScreen();

		/*-----------------------Handle button events--------------------*/
		if (xQueueReceive(pressedButtonQueue, &pressedButton, MENU_REFRESH_TIME) == pdPASS){
			if (pressedButton == RIGHT_BUTTON){
				if (backLightValue < 999){
					backLightValue += 100;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, backLightValue);
				}
			}
			else if (pressedButton == LEFT_BUTTON){
				if (backLightValue > 0){
					backLightValue -= 100;
					if (backLightValue < 0) backLightValue = 0;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, backLightValue);
				}
			}
			else if (pressedButton == CENETR_BUTTON){
				returnToMainMenu();
			}
		}
		PCD8544_clearScreen();
	}
}

void showContrastMenu(void *pvParameters){
	Buttons_e pressedButton;
	contrastValue = PCD8544_getContrast();

	for (;;){
		/*------------------------Draw Menu Items-----------------------*/
		PCD8544_GUI_displayMenuName("CONTRAST");

		PCD8544_GUI_drawProgressBar(4, 17, 76, 25, (contrastValue-50)*5);

		PCD8544_refreshScreen();

		/*-----------------------Handle button events--------------------*/

		if (xQueueReceive(pressedButtonQueue, &pressedButton, MENU_REFRESH_TIME) == pdPASS){
			if (pressedButton == RIGHT_BUTTON){
				if (contrastValue < 70){
					contrastValue += 1;
					PCD8544_setContrast(contrastValue);
				}
			}
			else if (pressedButton == LEFT_BUTTON){
				if (contrastValue > 50){
					contrastValue -= 1;
					PCD8544_setContrast(contrastValue);
				}
			}
			else if (pressedButton == CENETR_BUTTON){
				returnToMainMenu();
			}
		}
		PCD8544_clearScreen();
	}
}

void showHelpMenu(void *pvParameters){
	Buttons_e pressedButton;
	uint8_t helppageNumber = 0;

	for (;;){
		/*------------------------Draw Menu Items-----------------------*/
		PCD8544_GUI_displayMenuName("HELP");

		PCD8544_setFont(5);

		char tempBuffer[2];
		sprintf(tempBuffer, "%d", helppageNumber);
		PCD8544_printStringAlign(tempBuffer, 40, ALIGNMETN_RIGHT);

		if (helppageNumber == 0){
			PCD8544_printStringAlign("Enter or exit menus pressing the middle button.", 14, ALIGNMETN_LEFT);
		}
		else if (helppageNumber == 1){
			PCD8544_printStringAlign("Use left button to edit time and right one to adjust time.", 14, ALIGNMETN_LEFT);
		}
		else if (helppageNumber == 2){
			PCD8544_printStringAlign("Use right and left buttons to start/stop timer.", 14, ALIGNMETN_LEFT);
		}
		else if (helppageNumber == 3){
			PCD8544_printStringAlign("Press right button to increase backlight and left one to decrease it.", 14, ALIGNMETN_LEFT);
		}
		else if (helppageNumber == 4){
			PCD8544_printStringAlign("Do the same to adjust contrast.", 14, ALIGNMETN_LEFT);
		}
		else if (helppageNumber == 5){
			PCD8544_printStringAlign("Press the left button to rebbot when selected.", 14, ALIGNMETN_LEFT);
		}
		else if (helppageNumber == 6){
			PCD8544_printStringAlign("The system automatically restarts in times of hard fault.", 14, ALIGNMETN_LEFT);
		}
		PCD8544_setFont(1);
		PCD8544_refreshScreen();

		/*-----------------------Handle button events--------------------*/
		if (xQueueReceive(pressedButtonQueue, &pressedButton, MENU_REFRESH_TIME) == pdPASS){
			if (pressedButton == CENETR_BUTTON){
				returnToMainMenu();
			}
			else if (pressedButton == LEFT_BUTTON){
				if (helppageNumber > 0){
					helppageNumber--;
				}
			}
			else if (pressedButton == RIGHT_BUTTON){
				if (helppageNumber < 6){
					helppageNumber++;
				}
			}
		}

		PCD8544_clearScreen();
	}
}

void showRebootMenu(void *pvParameters){
	Buttons_e pressedButton;
	bool 			rebootState = false;

	for (;;){
		/*------------------------Draw Menu Items-----------------------*/
		PCD8544_GUI_displayMenuName("REBOOT");

		if (rebootState == false){
			PCD8544_printString("YES", 12, 25);
			PCD8544_setTextInvert(true);
			PCD8544_printString("NO", 57, 25);
			PCD8544_setTextInvert(false);
		}
		else if (rebootState == true){
			PCD8544_setTextInvert(true);
			PCD8544_printString("YES", 12, 25);
			PCD8544_setTextInvert(false);
			PCD8544_printString("NO", 57, 25);
		}

		PCD8544_drawVerticalLine(42, 25, 6);

		PCD8544_refreshScreen();

		/*-----------------------Handle button events--------------------*/
		if (xQueueReceive(pressedButtonQueue, &pressedButton, MENU_REFRESH_TIME) == pdPASS){
			 if (pressedButton == RIGHT_BUTTON){
				 rebootState = !rebootState;
			 }
			 else if (pressedButton == LEFT_BUTTON){
				 if (rebootState == true){
					 HAL_NVIC_SystemReset();
				 }
			 }
			 else if (pressedButton == CENETR_BUTTON){
				 returnToMainMenu();
			 }
		}
		PCD8544_clearScreen();
	}
}

void showTimerMenu(void *pvParameters){
	Buttons_e pressedButton;
	char			timerBuffer[10];

	for (;;){
		if (timerMenuCentiSeconds >= 100){
			timerMenuCentiSeconds = 0;
			timerMenuSeconds++;
			if (timerMenuSeconds >= 60){
				timerMenuSeconds = 0;
				timerMenuMinutes++;
				if (timerMenuMinutes >= 60){
					timerMenuCentiSeconds = 0;
					timerMenuSeconds = 0;
					timerMenuMinutes = 0;
				}
			}
		}
		/*------------------------Draw Menu Items-----------------------*/
		PCD8544_GUI_displayMenuName("TIMER");

		sprintf(timerBuffer, "%02d:%02d:%02d", timerMenuMinutes, timerMenuSeconds, timerMenuCentiSeconds);
		PCD8544_setFont(4);
		PCD8544_printStringAlign(timerBuffer, 25, ALIGNMETN_CENTER);
		PCD8544_setFont(1);
		PCD8544_refreshScreen();

		/*-----------------------Handle button events--------------------*/
		if (xQueueReceive(pressedButtonQueue, &pressedButton, pdMS_TO_TICKS(10)) == pdPASS){
				if (pressedButton == RIGHT_BUTTON){
					if (HAL_TIM_Base_GetState(&htim3) == HAL_TIM_STATE_BUSY){
						HAL_TIM_Base_Stop_IT(&htim3);
					}
					else {
						HAL_TIM_Base_Start_IT(&htim3);
					}
				}
				else if (pressedButton == LEFT_BUTTON){
					if (HAL_TIM_Base_GetState(&htim3) != HAL_TIM_STATE_BUSY){
						timerMenuCentiSeconds = 0;
						timerMenuSeconds = 0;
						timerMenuMinutes = 0;
					}
				}
				else if (pressedButton == CENETR_BUTTON){
					returnToMainMenu();
				}
		}
		PCD8544_clearScreen();
	}
}


void refreshIwdg(){
	HAL_IWDG_Refresh(&hiwdg);
}

/*---------------------------------------------------*/
void returnToMainMenu(){
	PCD8544_clearScreen();
	PCD8544_refreshScreen();
	vTaskResume(mainMenu_handle);
	vTaskDelete(NULL);
}

/*----------------Auto Generated Code-----------------*/
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  else if (htim->Instance == TIM3){
  	timerMenuCentiSeconds++;
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
