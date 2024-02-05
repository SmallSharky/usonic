/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
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

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

extern SD_HandleTypeDef hsd1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SD_DEFAULT_BLOCK_SIZE 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static volatile  UINT  WriteStatus = 0, ReadStatus = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{
  /* USER CODE BEGIN SD_initialize */
	Stat = STA_NOINIT;
	/* Check if the SD card is plugged in the slot */
	
	if (HAL_GPIO_ReadPin(SD_DET_GPIO_Port, SD_DET_Pin) != GPIO_PIN_RESET)
	{
		Stat = STA_NODISK;
		return Stat;
	}
	


	/* HAL SD initialization */
	
	Stat = HAL_SD_Init(&hsd1);
	
	

	/* Place for user code (may require BSP functions/defines to be added to the project) */

	return Stat;
  /* USER CODE END SD_initialize */
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  /* USER CODE BEGIN SD_status */
	int32_t ret;

	ret = (int32_t)((HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);

	return ret;

	/* Place for user code (may require BSP functions/defines to be added to the project) */
  /* USER CODE END SD_status */
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN SD_read */
	DRESULT res = RES_ERROR;
	uint32_t alignedAddr;
	/* Place for user code (may require BSP functions/defines to be added to the project) */

	
	
	if (HAL_SD_ReadBlocks(&hsd1, buff, (uint32_t)(sector), count, 1000) == BSP_ERROR_NONE) 
	//if (HAL_SD_ReadBlocks_DMA(&hsd1, buff, (uint32_t)(sector), count) == BSP_ERROR_NONE) 
	{
		/* wait until the read operation is finished */
		while (SD_status(0) != BSP_ERROR_NONE)
		{
		}
		res = RES_OK;
		
	}
	
	return res;
  /* USER CODE END SD_read */
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN SD_write */
	DRESULT res = RES_ERROR;

	/* Place for user code (may require BSP functions/defines to be added to the project) */

	//if (HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t*)buff, (uint32_t)(sector), count) == BSP_ERROR_NONE) 
	if (HAL_SD_WriteBlocks(&hsd1, (uint8_t*)buff, (uint32_t)(sector), count, 1000) == BSP_ERROR_NONE) 
	{
		/* wait until the read operation is finished */
		while (SD_status(0) != BSP_ERROR_NONE)
		{
		}
		res = RES_OK;
	}
	
	
	return res;

  /* USER CODE END SD_write */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  /* USER CODE BEGIN SD_ioctl */
	DRESULT res = RES_ERROR;
	HAL_SD_CardInfoTypeDef CardInfo;
	
	/* Place for user code (may require BSP functions/defines to be added to the project) */
	
	
	switch (cmd)
	{
		/* Make sure that no pending write process */
	case CTRL_SYNC :
		res = RES_OK;
		break;

		/* Get number of sectors on the disk (DWORD) */
	case GET_SECTOR_COUNT :
		HAL_SD_GetCardInfo(&hsd1, &CardInfo);
		*(DWORD*)buff = CardInfo.LogBlockNbr;
		res = RES_OK;
		break;

		/* Get R/W sector size (WORD) */
	case GET_SECTOR_SIZE :
		HAL_SD_GetCardInfo(&hsd1, &CardInfo);
		*(WORD*)buff = CardInfo.LogBlockSize;
		res = RES_OK;
		break;

		/* Get erase block size in unit of sector (DWORD) */
	case GET_BLOCK_SIZE :
		HAL_SD_GetCardInfo(&hsd1, &CardInfo);
		*(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
		res = RES_OK;
		break;

	default:
		res = RES_PARERR;
	}

	
	
	return res;

  /* USER CODE END SD_ioctl */
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN UserCode */
/* can be used to add code */
/* USER CODE END UserCode */