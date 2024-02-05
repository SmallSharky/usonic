/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.h
  * @brief   Header for sd_diskio.c module
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
#ifndef __SD_DISKIO_H
#define __SD_DISKIO_H

/* USER CODE BEGIN UserDefinitions */
/* Place for user definitions */
/* Includes ------------------------------------------------------------------*/

#define SD_TRANSFER_OK                0U
#define SD_TRANSFER_BUSY              1U

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */



/* USER CODE END UserDefinitions */

extern const Diskio_drvTypeDef  SD_Driver;

#endif /* __SD_DISKIO_H */
