/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	CLOCK = 0,
	TIMER,
	BACKLIGHT,
	CONTRAST,
	MCU_TEMP,
	HELP,
	REBOOT,
} MenuItems_e;

typedef enum {
	RIGHT_BUTTON,
	LEFT_BUTTON,
	CENETR_BUTTON,
} Buttons_e;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_BACKLIGHT_Pin GPIO_PIN_3
#define LCD_BACKLIGHT_GPIO_Port GPIOA
#define LEFT_BUTTON_INPUT_Pin GPIO_PIN_9
#define LEFT_BUTTON_INPUT_GPIO_Port GPIOA
#define CENTER_BUTTON_INPUT_Pin GPIO_PIN_10
#define CENTER_BUTTON_INPUT_GPIO_Port GPIOA
#define RIGHT_BUTTON_INPUT_Pin GPIO_PIN_11
#define RIGHT_BUTTON_INPUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define MENU_REFRESH_TIME pdMS_TO_TICKS(100)
#define BUTTON_DELAY_TIME pdMS_TO_TICKS(200)
#define IWDG_REFRESH_TIME pdMS_TO_TICKS(2000)

#define DEFAULT_BACKLIGHT_VALUE 999

#define MENU_ITEMS 7
#define PAGES 3
#define ITEMS_PER_PAGE 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
