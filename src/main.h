/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	void uPrintf(const char* fmt, ...); 
	void dPrintf(const char* fmt, ...);  
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USB_D2_Pin GPIO_PIN_2
#define USB_D2_GPIO_Port GPIOE
#define USB_D3_Pin GPIO_PIN_3
#define USB_D3_GPIO_Port GPIOE
#define USB_D4_Pin GPIO_PIN_4
#define USB_D4_GPIO_Port GPIOE
#define USB_D5_Pin GPIO_PIN_5
#define USB_D5_GPIO_Port GPIOE
#define USB_D6_Pin GPIO_PIN_6
#define USB_D6_GPIO_Port GPIOE
#define PC13_Pin GPIO_PIN_13
#define PC13_GPIO_Port GPIOC
#define PB2_Pin GPIO_PIN_2
#define PB2_GPIO_Port GPIOB
#define USB_D7_Pin GPIO_PIN_7
#define USB_D7_GPIO_Port GPIOE
#define USB_RXF_Pin GPIO_PIN_8
#define USB_RXF_GPIO_Port GPIOE
#define USB_TXE_Pin GPIO_PIN_9
#define USB_TXE_GPIO_Port GPIOE
#define USB_RD_Pin GPIO_PIN_10
#define USB_RD_GPIO_Port GPIOE
#define USB_WR_Pin GPIO_PIN_11
#define USB_WR_GPIO_Port GPIOE
#define TX2_EN_Pin GPIO_PIN_12
#define TX2_EN_GPIO_Port GPIOE
#define OTW2_Pin GPIO_PIN_14
#define OTW2_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOB
#define NCLK_Pin GPIO_PIN_12
#define NCLK_GPIO_Port GPIOB
#define BL_CTL_Pin GPIO_PIN_14
#define BL_CTL_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOD
#define BTN1_Pin GPIO_PIN_9
#define BTN1_GPIO_Port GPIOD
#define BTN1_EXTI_IRQn EXTI9_IRQn
#define BTN2_Pin GPIO_PIN_10
#define BTN2_GPIO_Port GPIOD
#define BTN3_Pin GPIO_PIN_11
#define BTN3_GPIO_Port GPIOD
#define BTN4_Pin GPIO_PIN_12
#define BTN4_GPIO_Port GPIOD
#define BTN5_Pin GPIO_PIN_13
#define BTN5_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOD
#define SD_DET_Pin GPIO_PIN_7
#define SD_DET_GPIO_Port GPIOC
#define W_CS_Pin GPIO_PIN_15
#define W_CS_GPIO_Port GPIOA
#define W_IRQ_Pin GPIO_PIN_0
#define W_IRQ_GPIO_Port GPIOD
#define W_IRQ_EXTI_IRQn EXTI0_IRQn
#define PD3_Pin GPIO_PIN_3
#define PD3_GPIO_Port GPIOD
#define TX_PD1_Pin GPIO_PIN_7
#define TX_PD1_GPIO_Port GPIOD
#define TX_EN1_Pin GPIO_PIN_8
#define TX_EN1_GPIO_Port GPIOB
#define TX_IN1_Pin GPIO_PIN_9
#define TX_IN1_GPIO_Port GPIOB
#define USB_D0_Pin GPIO_PIN_0
#define USB_D0_GPIO_Port GPIOE
#define USB_D1_Pin GPIO_PIN_1
#define USB_D1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
	
	
#define scr_height 336//240
#define scr_width 536//400
#define scr_buf scr_height * scr_width / 8
	
	
	
	
	
	
	
#define APP_OK                     0
#define APP_ERROR                  -1
#define APP_SD_UNPLUGGED           -2
#define APP_INIT                   1
	
	

	
	
#define SD_INSTANCES_NBR              1U

#ifndef SD_WRITE_TIMEOUT
#define SD_WRITE_TIMEOUT              250U
#endif

#ifndef SD_READ_TIMEOUT
#define SD_READ_TIMEOUT               100U
#endif

	
	/* Common Error codes */
#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED  -11
	
	
	
	
#define		SetBit(reg, x)		reg |= (1<<x)
#define		ClearBit(reg, x)	reg &= (~(1<<x))
#define		InvBit(reg, x)		reg ^= (1<<x)
#define		BitIsSet(reg, x)	((reg & (1<<x)) != 0)
#define		BitIsClear(reg, x)	((reg & (1<<x)) == 0)
#define		bit(x)				(1 << (x))


#define		hibyte(x)			(uint8_t)((x>>8) & 0xFF)
#define		lobyte(x)			(uint8_t)((x) & 0xFF)

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
