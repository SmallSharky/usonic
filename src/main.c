/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "aes.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "app_fatfs.h"
#include "i2c.h"
#include "icache.h"
#include "octospi.h"
#include "rng.h"
#include "rtc.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define PROGMEM 
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

#include "lcd_mem.h"

#include "filter.h"
#include "FreeSans12pt7b.h"


#include "BME280.h"
#include "arm_math.h"
#include <JPEGENC.h>


//wifi
#include "sl_wfx.h"
#include "sl_wfx_host_pin.h"
#include "lwip_app.h"
#include "demo_config.h"

extern void sl_wfx_process(void);


float32_t __attribute__((section(".ospi_data"))) firStateF32[10000];
arm_fir_instance_f32 Sf;

extern uint8_t sl_wfx_firmware[];


//#include "sl_wfx_host.h"
//extern uint8_t scan_count;
//extern scan_result_list_t scan_list[];


extern char wlan_ssid[]; 
extern char wlan_passkey[]; 
extern char softap_ssid[];
extern char softap_passkey[];



JPEGIMAGE __attribute__((section(".ospi_data")))  jpg;
JPEGENCODE __attribute__((section(".ospi_data"))) jpe;
int32_t iMCUCount, rc, iDataSize;


extern const uint8_t sl_wfx_firmware2[];

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_BUF_LEN 4096
#define FFT_BUF_LEN ADC_BUF_LEN << 1
#define ADC_FREQ 8421 //KHz
#define WATER_C 1488
#define UART_FIFO	128

#define PULSE_SIZE 1000
#define PACK_LEN 400
#define STEP_BRK 4

/* IS66WVH8M8BLL-100BLI ISSI memory */
/* Size of the HyperRAM */
#define OSPI_HYPERRAM_SIZE          24
#define OSPI_HYPERRAM_INCR_SIZE     256

#define OSPI_HYPERRAM_END_ADDR      (1 << OSPI_HYPERRAM_SIZE)
/* Size of buffers */


/* Global variables */
//__attribute__((section(".ospi_data"))) 
//float32_t __attribute__((section(".ospi_data"))) FFT_Input[FFT_BUF_LEN << 1];
//float32_t __attribute__((section(".ospi_data"))) FFT_Output[FFT_BUF_LEN << 1];

//float32_t __attribute__((section(".ospi_data"))) ResultFilter[10000];

uint8_t __attribute__((section(".ospi_data"))) jpgout[JPEG_FILE_BUF_SIZE];

uint16_t WifiReceived = 0;
bool WifiConnected = false;
bool HostConnecting = false;
extern uint8_t DispBuf[];
char __attribute__((section(".ospi_data"))) WifiPayload[1500];


arm_status status;
uint32_t ScreenShotIdx = 0;
bool NewStart = false;
bool wIcone = false;
bool SetupMode = false;


float32_t TempExt;
char UartRX[UART_FIFO];
uint16_t UartRxPos = 0;


//kolben 5L = 2400

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LD1(x) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, x^1)
#define LD2(x) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, x^1)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t ProcessStatus = 0;
extern volatile uint8_t wf200_interrupt_event;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void EnableMemMapped(void);
void DelayBlock_Calibration(void);

bool WriteIni();
bool ParseIni();
void stripSP(char *s);
void WifiTask(); 
void KeyAdj();
void UartTask();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t __attribute__((section(".ospi_data"))) buf[0x1000000];
FATFS fs; // file system
FIL fil; // File

FATFS* getFreeFs;
FRESULT res; // result
UINT br, bw; // File read/write count
DWORD fre_clust, fre_sect, tot_sect;

volatile uint32_t WifiTimeout;
volatile bool TxBusy = false;
uint8_t ReadByte;

uint16_t OscX;

bool TimeToSend;
char StringToSend[128];

char __attribute__((section(".ospi_data"))) buff[8192]; //prinft buffer

float AdcIn[ADC_BUF_LEN << 1];
float AdcOut[ADC_BUF_LEN << 1];


char __attribute__((section(".ospi_data")))  UsbReadBuf[256];
uint8_t UsbReadPointer = 0;
bool UsbReadStr = false;

struct Analog_t
{
	float h;
	float p;
	float t;
} AnalogData;

struct UartDma_t
{
	size_t head, tail;
	uint8_t RingBuff[128];
} UartDma;

	
uint32_t adc_buf[ADC_BUF_LEN];
volatile bool AdcDone = false;
float32_t FirstShift = 0;
bool FilterStart = false;

uint32_t AvgSettings;
uint8_t MenuPos = 0;

bool ButtonStart = false;
uint16_t ButtonCnts = 1;
volatile size_t TickStart;

bool FFtEn = true;

void UsbSend(uint8_t * data, size_t len)
{
	TickStart = HAL_GetTick();
	LD1(1);
	GPIOE->MODER = 0x89515555; //write
	while (len--)
	{
		GPIOE->ODR = (*data++) | 0x0C00; //write data and clock write up
		while ((GPIOE->IDR & GPIO_IDR_ID9))
		{
			if ((HAL_GetTick() - TickStart) > 10) break;
		}
		GPIOE->BRR = GPIO_BRR_BR11; //clock write down
	}
	GPIOE->BSRR = GPIO_BRR_BR11; //clock write up
	GPIOE->MODER = 0x89500000; //read
	LD1(0);
}

void UsbSendWord(uint16_t * data, size_t len)
{
	
	TickStart = HAL_GetTick();
	
	LD1(1);
	static uint8_t Lb, Hb;
	GPIOE->MODER = 0x89515555; //write
	while (len--)
	{
		Lb = *data & 0xFF;
		Hb = hibyte(*data); 
		
		GPIOE->ODR = Hb | 0x0C00; //write data and clock write up
		while ((GPIOE->IDR & GPIO_IDR_ID9))
		{
			if ((HAL_GetTick() - TickStart) > 10) break;
		}
		GPIOE->BRR = GPIO_BRR_BR11; //clock write down
		
		GPIOE->ODR = Lb | 0x0C00; //write data and clock write up
		while ((GPIOE->IDR & GPIO_IDR_ID9)) 
		{
			if ((HAL_GetTick() - TickStart) > 10) break;
		}
		GPIOE->BRR = GPIO_BRR_BR11; //clock write down
		data++;
	
	}
	GPIOE->BSRR = GPIO_BRR_BR11; //clock write up
	GPIOE->MODER = 0x89500000; //read
	LD1(0);
}

void UsbSendDword(uint32_t * data, size_t len)
{
	TickStart = HAL_GetTick();
	LD1(1);
	static uint8_t b[4];
	GPIOE->MODER = 0x89515555; //write
	while (len--)
	{
		b[0] = *data >> 8;
		b[1] = *data & 0xFF;
		b[2] = *data >> 24;
		b[3] = *data >> 16;

		
		
		GPIOE->ODR = b[0] | 0x0C00; //write data and clock write up
		while ((GPIOE->IDR & GPIO_IDR_ID9))
		{
			if ((HAL_GetTick() - TickStart) > 10) break;
		}
		GPIOE->BRR = GPIO_BRR_BR11; //clock write down
		
		GPIOE->ODR = b[1] | 0x0C00; //write data and clock write up
		while ((GPIOE->IDR & GPIO_IDR_ID9))
		{
			if ((HAL_GetTick() - TickStart) > 10) break;
		}
		GPIOE->BRR = GPIO_BRR_BR11; //clock write down
		
		GPIOE->ODR = b[2] | 0x0C00; //write data and clock write up
		while ((GPIOE->IDR & GPIO_IDR_ID9))
		{
			if ((HAL_GetTick() - TickStart) > 10) break;
		}
		GPIOE->BRR = GPIO_BRR_BR11; //clock write down
		
		GPIOE->ODR = b[3] | 0x0C00; //write data and clock write up
		while ((GPIOE->IDR & GPIO_IDR_ID9))
		{
			if ((HAL_GetTick() - TickStart) > 10) break;
		}
		GPIOE->BRR = GPIO_BRR_BR11; //clock write down
			
		
		data++;
	
	}
	GPIOE->BSRR = GPIO_BRR_BR11; //clock write up
	GPIOE->MODER = 0x89500000; //read
	LD1(0);
}

float gaussian(float x, float mu, float sig) {
	return 1.0 / (sqrt(2.0 * PI) * sig) * exp(-pow((x - mu) / sig, 2.0) / 2);
}

void FillOutputBuffer()
{
	float mu = Coe.PackLen>>1;
	float sig = Coe.PackLen;
	float k = (float)Coe.Freq / gaussian(mu, mu, sig);
	//set tx len
	for (uint8_t i = 0; i < 15; i++)
	{
		TimTxPack[i] = k * gaussian(i, mu, sig);
	}
	TimTxPack[15] = 0;
}

void uPrintf(const char* fmt, ...) {
	
	static uint16_t bts;
	va_list args;
	va_start(args, fmt);
	bts = vsnprintf(buff, sizeof(buff), fmt, args);
	va_end(args);	
	TxBusy = true;
	GPIOE->MODER |= 0x5555; //set TX
	UsbSend((uint8_t*)buff, bts);
	GPIOE->MODER &= 0xFFFF0000; //set RX

}


void dPrintf(const char* fmt, ...) {

	uint16_t bts;
	va_list args;
	va_start(args, fmt);
	bts = vsnprintf(buff, sizeof(buff), fmt, args);
	va_end(args);	
	uint8_t *p = (uint8_t*)buff;
	
	while (bts--) 
	{
		LCD_write(*p++);		
	}

	
}




//
//
///* initialize decimator for in-phase channel */
//int Dec_Init(void)
//{
//
//
//	/* enable clock for dma */
//	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
//	/* enable demod */
//	RCC->APB2ENR |= RCC_APB2ENR_DFSDM1EN;
//
//	/* use dma2c7 for pushing data into I decimator */
//	DMA2_Channel7_NS->CCR = DMA_CCR_MEM2MEM | DMA_CCR_DIR | DMA_CCR_MINC |
//			DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
//	/* set destination register */
//	DMA2_Channel7_NS->CPAR = (uint32_t)&DFSDM1_Channel0->CHDATINR;
//
//	/* use dma2c2 for pushing data into Q decimator */
//	DMA2_Channel2_NS->CCR = DMA_CCR_MEM2MEM | DMA_CCR_DIR | DMA_CCR_MINC |
//			DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
//	/* set destination register */
//	DMA2_Channel2_NS->CPAR = (uint32_t)&DFSDM1_Channel1->CHDATINR;
//
//	/* use dma1c4 for fetching the I data from the decimator */
//	DMA1_c ->CSELR = (DMA1->CSELR & ~DMA_CSELR_C4S) | DMA1_CSELR_C4S_DFSDM0;
//	/* configure dma fetching data out of I decimator */
//	DMA1_Channel4_NS->CCR = DMA_CCR_TCIE | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | 
//        DMA_CCR_MINC;
//	/* set source register */
//	DMA1_Channel4_NS->CPAR = (uint32_t)&DFSDM1_Filter0_NS->FLTRDATAR;
//
//	/* use dma1c5 for fetching the Q data from the decimator */
//	DMA1->CSELR = (DMA1->CSELR & ~DMA_CSELR_C5S) | DMA1_CSELR_C5S_DFSDM1;
//	/* configure dma fetching data out of Q decimator */
//	DMA1C5->CCR = DMA_CCR_TCIE | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 |
//			DMA_CCR_MINC;
//	/* set source register */
//	DMA1C5->CPAR = (uint32_t)&DFSDMF1->RDATAR;
//
//	/* set interrupt priority */
//	NVIC_SETINTPRI(STM32_INT_DMA1C4, INT_PRI_DEC);
//
//	/* this is prepared for only one decimation rate */
//	assert(DEC_DECIMATION_RATE == 50,
//		"unsupported decimation factor",
//		DEC_DECIMATION_RATE);
//
//	/* enable interface */
//	DFSDMC0->CHCFGR1 |= DFSDM_CHCFGR1_DFSDMEN;
//
//	/* 0th channel: I data input */
//	/* configure for single data input */
//	DFSDMC0->CHCFGR1 |= DFSDM_CHCFGR1_DATMPX_1;
//	/* configure offset and bit shift  */
//	DFSDMC0->CHCFGR2 |= 8 << LSB(DFSDM_CHCFGR2_DTRBS);
//	/* enable channel */
//	DFSDMC0->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;
//
//	/* 1st channel: Q data input */
//	/* configure for single data input */
//	DFSDMC1->CHCFGR1 |= DFSDM_CHCFGR1_DATMPX_1;
//	/* configure offset and bit shift  */
//	DFSDMC1->CHCFGR2 |= 8 << LSB(DFSDM_CHCFGR2_DTRBS);
//	/* enable channel */
//	DFSDMC1->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;
//
//	/* 0th Filter: mapped to channel 0 (I samples), decimation by 50, sinc^4
//	 * filter, bit growth = N*log2(R) = 3 * log2(50) ~= 18, output data width =
//	 * input data width + bit growth = 14b + 18b = 32b, but since the output
//	 * register can only handle 24 bit data we need to shift by 6 (done in
//	 * input channel configuration) */
//	/* disable block */
//	DFSDMF0->CR1 &= ~DFSDM_CR1_DFEN;
//	/* enable fast conversion, enable dma requests  */
//	DFSDMF0->CR1 = DFSDM_CR1_FAST | DFSDM_CR1_RDMAEN;
//	/* sinc^3 filter, decimation by 50, no integration */
//	DFSDMF0->FCR = DFSDM_FCR_FORD_1 | DFSDM_FCR_FORD_0 |
//        (DEC_DECIMATION_RATE - 1) << LSB(DFSDM_FCR_FOSR) | 
//        0 << LSB(DFSDM_FCR_IOSR);
//	/* select regular channel 0, continuous conversion */
//	DFSDMF0->CR1 |= DFSDM_CR1_RCONT | 0 << LSB(DFSDM_CR1_RCH);
//	/* enable filtering */
//	DFSDMF0->CR1 |= DFSDM_CR1_DFEN;
//
//	/* 1st Filter: mapped to channel 1 (Q samples), decimation by 50, sinc^3
//	 * filter, bit growth = N*log2(R) = 3 * log2(50) ~= 18, output data width =
//	 * input data width + bit growth = 14b + 18b = 32b, but since the output
//	 * register can only handle 24 bit data we need to shift by 8 (done in
//	 * input channel configuration) */
//	/* disable block */
//	DFSDMF1->CR1 &= ~DFSDM_CR1_DFEN;
//	/* enable fast conversion, enable dma requests  */
//	DFSDMF1->CR1 = DFSDM_CR1_FAST | DFSDM_CR1_RDMAEN;
//	/* sinc^3 filter, decimation by 50, no integration */
//	DFSDMF1->FCR = DFSDM_FCR_FORD_1 | DFSDM_FCR_FORD_0  | 
//        (DEC_DECIMATION_RATE - 1) << LSB(DFSDM_FCR_FOSR) |
//        0 << LSB(DFSDM_FCR_IOSR);
//	/* select regular channel 0, continuous conversion */
//	DFSDMF1->CR1 |= DFSDM_CR1_RCONT | 1 << LSB(DFSDM_CR1_RCH);
//	/* enable filtering */
//	DFSDMF1->CR1 |= DFSDM_CR1_DFEN;
//
//	/* start filter operation */
//	DFSDMF0->CR1 |= DFSDM_CR1_RSWSTART;
//	DFSDMF1->CR1 |= DFSDM_CR1_RSWSTART;
//	/* initialize filter, this needs to be done because filter is not willing to
//	 * output any data before it's integrators and combs are filled (decimation
//	 * factor * filter order samples are needed) */
//	for (int i = 0; i < DEC_DECIMATION_RATE * 50; i++) {
//		DFSDMC1->CHDATINR = 0; DFSDMC0->CHDATINR = 0; 
//	}
//
//	/* sanity check */
//	assert(sizeof(int32_t) == sizeof(float), 
//		"int32_t must have the same size as the float for this code to work",
//		0);
//
//	/* exit critical section */
//	Critical_Exit();
//
//	/* release the device */
//	Sem_Release(&dec_sem);
//
//	/* report status */
//	return EOK;
//}
//
//
//





































void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_CLEAR_PEFLAG(huart);

	if (huart == &huart1)
	{
		HAL_UART_Receive_DMA(&huart1, UartDma.RingBuff, 128);
	}
	
}


bool GetUart(uint8_t* bt)
{
	
	UartDma.tail = 128 - huart1.hdmarx->Instance->CNDTR;
	if (UartDma.head != UartDma.tail)
	{
		*bt = UartDma.RingBuff[UartDma.head++];
		if (UartDma.head == 128) UartDma.head = 0;
		return true;
	}
	return false;
}




uint32_t scan_files(char* path)
{
	FRESULT res;
	DIR dir;
	UINT i;
	char *p;
	static FILINFO fno;
	uint32_t FileIdx = 0;
	char *ptr;

	res = f_opendir(&dir, path); /* Open the directory */
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno); /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR) {
				/* It is a directory */
				i = strlen(path);
				sprintf(&path[i], "/%s", fno.fname);
				//uPrintf("%s\r\n", path);
				res = scan_files(path); /* Enter the directory */
				if (res != FR_OK) break;
				path[i] = 0;
			}
			else {
				/* It is a file. */
				//uPrintf("%s/%s\r\n", path, fno.fname);
			}
		}
		p = strstr(fno.fname + 1, ".jpg");
		if (p)
		{
			p -= 6;
			FileIdx = strtol(p, &ptr, 10);
		}
		f_closedir(&dir);
	}

	return FileIdx;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef* hsd)
{
	__asm__("NOP");
	
}
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)
	{
		wf200_interrupt_event = 1;
	}
	
}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)
	{

	}
	
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) 
	{
		//GPIOB->BSRR = GPIO_BRR_BR8; //TX STOP
	}
	else 
	if (htim->Instance == TIM7) 
	{
		//GPIOB->BRR = GPIO_BRR_BR8; //TX START
	}
	
	
	
}


void  OneMeasure()
{
		
	HAL_TIM_Base_Stop(&htim4);
	TIM4->ARR = Coe.Freq;
	//	TIM4->CCR1 = TIM4->ARR >> 1;
	TIM4->CNT = 0;

	HAL_GPIO_WritePin(TX_PD1_GPIO_Port, TX_PD1_Pin, 0);
	GPIOB->BRR = GPIO_BRR_BR8; //TX ENABLE
	
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_buf, Glass.AdcSize >> 1);
	
	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, (uint32_t*) TimTxPack, 16);	

	
	while (!AdcDone) ;
	AdcDone = false;
	//	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
				
	for (uint16_t x = 0; x < Glass.AdcSize >> 1; x++)
	{
		uint16_t Hb = adc_buf[x] >> 16;
		uint16_t Lb = adc_buf[x] & 0xFFFF;
		
		AdcIn[x << 1] = (float32_t)((float32_t)(Lb) - (float32_t)2048.0) / (float32_t)2048.0;
		AdcIn[(x << 1) + 1] = (float32_t)((float32_t)(Hb) - (float32_t)2048.0) / (float32_t)2048.0;
	}
}



uint16_t FullScan()
{

#define SCAN_LEN 3000

	static uint32_t __attribute__((section(".ospi_data"))) ScanBuffer[SCAN_LEN >> 1];
	static float32_t __attribute__((section(".ospi_data"))) AdcData[SCAN_LEN];
	float32_t AbsMax;
	
	HAL_GPIO_WritePin(TX_EN1_GPIO_Port, TX_EN1_Pin, 1);
	HAL_TIM_Base_Stop(&htim4);
	TIM4->ARR = 55;
	TIM4->CCR1 = TIM4->ARR >> 1;
	TIM4->CNT = 0;
	HAL_GPIO_WritePin(TX_PD1_GPIO_Port, TX_PD1_Pin, 0);
	GPIOB->BRR = GPIO_BRR_BR8; //TX ENABLE
	
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ScanBuffer, SCAN_LEN >> 1);

	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, (uint32_t*) TimTxPack, 16);	

	
	while (!AdcDone) ;
	AdcDone = false;

	
	for (uint16_t x = 0; x < SCAN_LEN >> 1; x++) 
	{
		KeyAdj();
		uint16_t Hb = ScanBuffer[x] >> 16;
		uint16_t Lb = ScanBuffer[x] & 0xFFFF;
		AdcData[x << 1] = (float32_t)((float32_t)(Lb) - (float32_t)2048.0) / (float32_t)2048.0;
		AdcData[(x << 1) + 1] = (float32_t)((float32_t)(Hb) - (float32_t)2048.0) / (float32_t)2048.0;
	}

	
	arm_fir_init_f32(&Sf, FirCoe, filterCoeff, firStateF32, SCAN_LEN);
	arm_fir_f32(&Sf, AdcData, AdcData, SCAN_LEN);	
	
	LCD_clearBuf();
	
	for (uint16_t x = 0; x < SCAN_LEN - FirCoe / 2; x++) 
	{
		KeyAdj();
		LCD_Pixel(x / (SCAN_LEN / scr_width), 100 - AdcData[x + FirCoe / 2] * 100, 1);
		AdcData[x] = powf(AdcData[x + FirCoe / 2], 2);
	}
	

	LCD_line(Coe.AbsBreak1 / (SCAN_LEN / scr_width), 30, Coe.AbsBreak1 / (SCAN_LEN / scr_width), 70, 1); //Start

	
	LCD_line(Coe.AbsBreak2 / (SCAN_LEN / scr_width), 20, Coe.AbsBreak2 / (SCAN_LEN / scr_width), 80, 1); //Cut off
	
	
	
	
	arm_fir_init_f32(&Sf, FirOutCoe, FilterOut, firStateF32, SCAN_LEN);
	KeyAdj();
	arm_fir_f32(&Sf, AdcData, AdcData, SCAN_LEN);
	
	bool FlasgStop = false;
	
	AbsMax = 0;
	for (uint16_t x = 0; x < SCAN_LEN - FirCoe / 2 - FirOutCoe / 2; x++) 
	{
		KeyAdj();
		AdcData[x] = sqrtf(AdcData[x + FirOutCoe / 2]);
		if (AdcData[x] > AbsMax && !FlasgStop)
		{
			AbsMax = AdcData[x];
			if (x < Coe.AbsBreak2) Glass.FirstPos = x;
		}
		else if (AdcData[x] < AbsMax - Coe.AbsMin2) FlasgStop = true;
		LCD_Pixel(x / (SCAN_LEN / scr_width), 50 - AdcData[x + FirOutCoe / 2] * 100 / AbsMax, 1);
	}
	
	
	
	//arm_absmax_f32(AdcData, Coe.AbsBreak2, &AbsMax, &Glass.FirstPos);		

	
	Glass.FirstVal = AdcData[Glass.FirstPos];
	
	for (uint16_t x = 0; x < scr_width; x++) 
	{
		LCD_Pixel(x, + scr_height - AdcData[x + Glass.FirstPos - 100] * 100 / Glass.FirstVal, 1);
	}
	
	LCD_line(100, 230, 100, 250, 1); //first peak

		
	Glass.AdcSize = (uint16_t)(ceil((Glass.FirstPos + 200) * 0.01) * 100);

	double Dia = ((double)(Glass.FirstPos) - Coe.PackLen) / ADC_FREQ * WATER_C;
	double PulseL = (double)(Glass.FirstPos - Coe.PackLen) / ADC_FREQ * 1000.0f;

	
	
	LCD_setCursor(220, 127);
	if (MenuPos == 0) dPrintf(">");
	LCD_setCursor(233, 130);
	dPrintf("Gls: %.4f", Coe.AbsMin2);
	
	LCD_setCursor(220, 147);
	if (MenuPos == 1) dPrintf(">");
	LCD_setCursor(233, 150);
	dPrintf("Stt: %.0f", Coe.AbsBreak1);

	LCD_setCursor(400, 127);
	if (MenuPos == 2) dPrintf(">");
	LCD_setCursor(413, 130);
	dPrintf("PS: %u", Coe.PackLen);

	LCD_setCursor(400, 147);
	if (MenuPos == 3) dPrintf(">");
	LCD_setCursor(413, 150);
	dPrintf("Brk: %.0f", Coe.AbsBreak2);
	
	LCD_setCursor(220, 180);
	dPrintf("TOF: %.2f us", PulseL);
	
//	LCD_setCursor(410, 290);
//	dPrintf("Dst: %u", Glass.SecondPos - Glass.FirstPos);
	
	LCD_setCursor(220, 230);
	dPrintf("1st    Peak: %u", Glass.FirstPos);
	LCD_setCursor(220, 260);
//	dPrintf("2nd   Peak: %u", Glass.SecondPos);
//	LCD_setCursor(220, 290);
	dPrintf("ADC Size: %u", Glass.AdcSize); 
	
	LCD_setCursor(410, 230);
	dPrintf("Amp: %.2f", Glass.FirstVal);
//	LCD_setCursor(410, 260);
//	dPrintf("Amp: %.2f", Glass.SecondVal);
	
	LCD_setCursor(410, 180);
	dPrintf("D: %.0f mm", Dia);
	
	
	LCD_update();
	Glass.FirstPos -= 100;
	Glass.SecondPos -= 200;
	return 0;
}



uint16_t FullScan__()
{

#define SCAN_LEN 3000

	static uint32_t __attribute__((section(".ospi_data"))) ScanBuffer[SCAN_LEN >> 1];
	static float32_t __attribute__((section(".ospi_data"))) AdcData[SCAN_LEN];
	float32_t AbsMax;
	
	HAL_GPIO_WritePin(TX_EN1_GPIO_Port, TX_EN1_Pin, 1);
	HAL_TIM_Base_Stop(&htim4);
	TIM4->ARR = 55;
	TIM4->CCR1 = TIM4->ARR >> 1;
	TIM4->CNT = 0;
	HAL_GPIO_WritePin(TX_PD1_GPIO_Port, TX_PD1_Pin, 0);
	GPIOB->BRR = GPIO_BRR_BR8; //TX ENABLE
	
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ScanBuffer, SCAN_LEN >> 1);

	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, (uint32_t*) TimTxPack, 16);	

	
	while (!AdcDone) ;
	AdcDone = false;

	
	for (uint16_t x = 0; x < SCAN_LEN >> 1; x++) 
	{
		KeyAdj();
		uint16_t Hb = ScanBuffer[x] >> 16;
		uint16_t Lb = ScanBuffer[x] & 0xFFFF;
		AdcData[x << 1] = (float32_t)((float32_t)(Lb) - (float32_t)2048.0) / (float32_t)2048.0;
		AdcData[(x << 1) + 1] = (float32_t)((float32_t)(Hb) - (float32_t)2048.0) / (float32_t)2048.0;
	}

	
	arm_fir_init_f32(&Sf, FirCoe, filterCoeff, firStateF32, SCAN_LEN);
	arm_fir_f32(&Sf, AdcData, AdcData, SCAN_LEN);	
	
	LCD_clearBuf();
	
	for (uint16_t x = 0; x < SCAN_LEN - FirCoe / 2; x++) 
	{
		KeyAdj();
		LCD_Pixel(x / (SCAN_LEN / scr_width), 100 - AdcData[x] * 100, 1);
		AdcData[x] = powf(AdcData[x + FirCoe / 2], 2);
	}
	
	arm_fir_init_f32(&Sf, FirOutCoe, FilterOut, firStateF32, SCAN_LEN);
	KeyAdj();
	arm_fir_f32(&Sf, AdcData, AdcData, SCAN_LEN);
	
	for (uint16_t x = 0; x < SCAN_LEN - FirCoe / 2 - FirOutCoe / 2; x++) 
	{
		KeyAdj();
		AdcData[x] = sqrtf(AdcData[x + FirOutCoe / 2]);
		LCD_Pixel(x / (SCAN_LEN / scr_width), 50 - AdcData[x] * 100, 1);
	}
	
	arm_absmax_no_idx_f32(AdcData, SCAN_LEN, &AbsMax);
	
	Glass.FirstVal = 0;
	for (uint16_t x = 0; x < SCAN_LEN; x++) 
	{
		if (AdcData[x] > Glass.FirstVal) 
		{
			if (AdcData[x] > (AbsMax / Coe.AbsMin1)) 
			{
				Glass.FirstVal = AdcData[x];
				Glass.FirstPos = x;
			}
		}
		else if (AdcData[x] < Glass.FirstVal - AbsMax / Coe.AbsBreak1) break;
	}
	
	for (uint16_t x = 0; x < scr_width; x++) 
	{
		LCD_Pixel(x, 210 - AdcData[x + Glass.FirstPos - 100] * 80 / Glass.FirstVal, 1);
	}
	
	LCD_line(100, 120, 100, 150, 1); //first peak

	
	Glass.SecondVal = 0;
	uint16_t StepCnt = 0;
	uint16_t SecondP;
	for (uint16_t x = Glass.FirstPos + PACK_LEN; x < SCAN_LEN; x++) 
	{		
		
		if (AdcData[x] > (AbsMax / Coe.AbsMin2)) 
		{
			Glass.SecondVal = AdcData[x];
			Glass.SecondPos = x;
			break;
		}
		
//		StepCnt++;
//		if (AdcData[x] > Glass.SecondVal + Coe.AbsBreak2 / 10000) 
//		{
//			if (AdcData[x] > (AbsMax / Coe.AbsMin2)) 
//			{
//				Glass.SecondVal = AdcData[x];
//				Glass.SecondPos = x;
//				StepCnt = 0;
//				//				uPrintf("SP: %u Sv: %f\r\n", Glass.SecondPos, Glass.SecondVal);
//			}
//		}
//		else 
//		{
////			if (AdcData[x] < Glass.SecondVal - AbsMax / Coe.AbsBreak2 || StepCnt > STEP_BRK) 
//			{		
//				//uPrintf(">>%f\r\n", AbsMax / (Coe.AbsBreak2 * 100));
//				SecondP = x;
//				//				uPrintf("BP: %u %u\r\n", x, StepCnt);
//				//				
//				//				uPrintf(">>%f / %f / %f\r\n",
//				//					AdcData[x], 
//				//					Glass.SecondVal - AbsMax / Coe.AbsBreak2,
//				//					Coe.AbsBreak2);
//				break;
//			}
//			
//		}
	}
	
	for (uint16_t x = 0; x < scr_width; x++) 
	{
		LCD_Pixel(x, 330 - AdcData[x + Glass.SecondPos - 100] * 80 / Glass.SecondVal, 1);
	}
	
	SecondP = SecondP - Glass.SecondPos;
	
	LCD_line(100 + SecondP, 250, 100 + SecondP, 270, 1); //second pos
	
	LCD_line(100, 240, 100, 280, 1); //second peak

	uint16_t hPosS = (Glass.SecondVal - AbsMax / Coe.AbsBreak2) * (80 / Glass.SecondVal);
	//		uPrintf("hP: %u / %f / %f\r\n",
	//		hPosS, 
	//		AbsMax,
	//		80 / Glass.SecondVal);
	
	//LCD_line(40, 330 - hPosS, 140, 330 - hPosS, 1); //break
	
	
	uint16_t hPosMin =  (AbsMax / Coe.AbsMin2) * 80 / Glass.SecondVal;
	LCD_line(10, 330 - hPosMin, 170, 330 - hPosMin, 1); //min
	
	
	
	
	
		
	Glass.AdcSize = (uint16_t)(ceil((Glass.SecondPos + 100) * 0.01) * 100);

	double Dia = ((double)(Glass.SecondPos - Glass.FirstPos) / 2.0f) / ADC_FREQ * WATER_C;
	double PulseL = (double)(Glass.SecondPos - Glass.FirstPos) / ADC_FREQ * 1000.0f;

	
	
	LCD_setCursor(220, 127);
	if (MenuPos == 0) dPrintf(">");
	LCD_setCursor(233, 130);
	dPrintf("Min: %.1f", Coe.AbsMin2);
	
	LCD_setCursor(220, 147);
	if (MenuPos == 1) dPrintf(">");
	LCD_setCursor(233, 150);
	dPrintf("Brk: %.1f", Coe.AbsBreak2);

	LCD_setCursor(410, 127);
	if (MenuPos == 2) dPrintf(">");
	LCD_setCursor(423, 130);
	dPrintf("PS: %u", Coe.PackLen);
	
	//LCD_setCursor(410, 170);
	
	LCD_setCursor(220, 180);
	dPrintf("TOF: %.2f us", PulseL);
	
	LCD_setCursor(410, 290);
	dPrintf("Dst: %u", Glass.SecondPos - Glass.FirstPos);
	
	LCD_setCursor(220, 230);
	dPrintf("1st    Peak: %u", Glass.FirstPos);
	LCD_setCursor(220, 260);
	dPrintf("2nd   Peak: %u", Glass.SecondPos);
	LCD_setCursor(220, 290);
	dPrintf("ADC Size: %u", Glass.AdcSize); 
	
	LCD_setCursor(410, 230);
	dPrintf("Amp: %.2f", Glass.FirstVal);
	LCD_setCursor(410, 260);
	dPrintf("Amp: %.2f", Glass.SecondVal);
	
	LCD_setCursor(410, 180);
	dPrintf("D: %.0f mm", Dia);
	
	
	LCD_update();
	Glass.FirstPos -= 100;
	Glass.SecondPos -= 200;
	return 0;
}




void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	//GPIOB->BSRR = GPIO_BRR_BR8; //TX FREE
	HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
	
}


uint16_t CoreProc()
{
	float32_t AbsMax;
	uint16_t MaxPos = 0, MaxPos2;
	float32_t AdcValue;
	float MaxCor;
	uint16_t StepCnt = 0;
	
	OneMeasure();
			
	arm_fir_init_f32(&Sf, FirCoe, filterCoeff, firStateF32, Glass.AdcSize);
	arm_fir_f32(&Sf, AdcIn, AdcIn, Glass.AdcSize);

			
	for (uint16_t x = (uint16_t)Coe.AbsBreak1; x < Glass.AdcSize; x++) 
	{
		AdcIn[x] = AdcIn[x + FirCoe / 2];
		AdcOut[x] = powf(AdcIn[x], 2);
	}
			
	arm_fir_init_f32(&Sf, FirOutCoe, FilterOut, firStateF32, Glass.AdcSize);
	arm_fir_f32(&Sf, AdcOut, AdcOut, Glass.AdcSize);
			
	AbsMax = 0;
	for (uint16_t x = (uint16_t)Coe.AbsBreak1; x < Glass.AdcSize; x++) 
	{
		AdcOut[x] = sqrtf(AdcOut[x + FirOutCoe / 2]);
		if (AdcOut[x] > AbsMax) 
		{
			MaxPos = x;
			AbsMax = AdcOut[x];
		}
		else if (AdcOut[x] < AbsMax - Coe.AbsMin2) 
		{
			break;
		}

	
	}
						
	
	

//	
//	MaxCor = 0;
//	for (uint16_t x = 0; x < 512; x++) 
//	{
//		AdcValue = AdcOut[x + Glass.FirstPos];
//		if (AdcValue > MaxCor) 
//		{
//			if (AdcValue > (AbsMax / Coe.AbsMin1))
//			{
//				MaxCor = AdcValue;
//				MaxPos = x;
//			}
//		}
//		else if (AdcValue < MaxCor - AbsMax / Coe.AbsBreak1) break;
//	}
//	
//	MaxCor = 0;
//	MaxPos2 = 0;
//	for (uint16_t x = 0; x < 512; x++) 
//	{
//		AdcValue = AdcOut[x + Glass.SecondPos];
//		
////		if (AdcValue > MaxCor + Coe.AbsBreak2 / 10000) 
////		{
//			if (AdcValue > (AbsMax / Coe.AbsMin2))
//			{
//				StepCnt = 0;
//				MaxCor = AdcValue;
//				MaxPos2 = x;
//				break;
//			}
////		}
//		//else if (AdcValue < MaxCor - AbsMax / Coe.AbsBreak2 || StepCnt > STEP_BRK) 
////		else
////		{
////			break;	
////		}
//		
//	}
	
//	if (!MaxPos2) return 0;
	
	if (Glass.Delay) HAL_Delay(Glass.Delay - 1);
	
	return MaxPos;
	
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		//	__disable_irq();
		//	SCB->VTOR = 0x8000000;
		//	__DSB();
		//	__enable_irq();
	

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
  MX_OCTOSPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ICACHE_Init();
  MX_SDMMC1_SD_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_RTC_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_AES_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	  
	HAL_ICACHE_Enable();
	
	uint32_t i = 0;
	uint32_t * addr;
	uint32_t address = 0x3FFF00;
	uint16_t index;
	
	i = 1;
	
	arm_cfft_radix4_instance_f32 S; /* ARM CFFT module */
	arm_cfft_instance_f32 cfft_instance_ptr;
	
	arm_rfft_fast_instance_f32 	Sint;
	
	float32_t maxValue; /* Max FFT value is stored here */
	uint32_t maxIndex; /* Index in Output array where max value is */
	
	FirCoe = sizeof(filterCoeff) >> 2;
	FirOutCoe = sizeof(FilterOut) >> 2;
		
	//uint32_t rrr = sizeof(float32_t) * (16384 + FirCoe - 1);
	
	Coe.AbsMin1 = 2.0f;
	Coe.AbsBreak1 = 20.0f;
	Coe.AbsMin2 = 10.0f;
	Coe.AbsBreak2 = 50.0f;
	Coe.PackLen = 4; 
	Coe.Freq = 55;
	Glass.Delay = 0;

	
	LD1(1);
	LD2(1);
	GPIOE->OSPEEDR = -1;

	GPIOE->MODER = 0x89500000; //read

	__IO uint8_t *mem_addr;
	
	
	OSPI_HyperbusCmdTypeDef sCommand;
	OSPI_MemoryMappedTypeDef sMemMappedCfg;
	
	sCommand.AddressSpace = HAL_OSPI_MEMORY_ADDRESS_SPACE;
	sCommand.AddressSize  = HAL_OSPI_ADDRESS_32_BITS;
	sCommand.DQSMode      = HAL_OSPI_DQS_ENABLE;
	sCommand.Address      = 0;
	sCommand.NbData       = 1;
  
	if (HAL_OSPI_HyperbusCmd(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}
	
	sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
  
	if (HAL_OSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK)
	{
		Error_Handler();
	}

	sprintf(wlan_ssid, "D922");
	sprintf(wlan_passkey, "77777701");
	sprintf(softap_ssid, "-");
	sprintf(softap_passkey, "-");

	
	LCD_clear();
	LCD_update();	
	
	HAL_Delay(500);
	
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	
	for (i = 0; i < 255; i++)
	{
		TIM15->CCR1 = i + 1;
		HAL_Delay(2);
	}
	
	LD1(0);
	LD2(0);	

	LCD_clear();

	LCD_setFont(&FreeSans12pt7b);

	LCD_setCursor(0, 20);
	dPrintf("Screenshot Analysis   Calibration     Setup       Start");
	
	

	
	LCD_update();

	HAL_Delay(100);

	
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED);
	
	GPIOB->BRR = GPIO_BRR_BR8; //TX START
	HAL_GPIO_WritePin(TX_PD1_GPIO_Port, TX_PD1_Pin, 0);
	
	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, (uint32_t*) TimTxPack, 16);
	         
	

	uPrintf("\r\nReady\r\n");

	HAL_UART_Receive_DMA(&huart1, UartDma.RingBuff, 128);
	
	
	
	LCD_setCursor(0, 60);
	
	res = f_mount(&fs, "", 1);
	if (!res)
	{
		res = f_getfree("", &fre_clust, &getFreeFs);	
		
		tot_sect = (fs.n_fatent - 2) * fs.csize;
		fre_sect = fre_clust * fs.csize;

		/* Print the free space (assuming 512 bytes/sector) */
		uPrintf("%10lu KB total drive space.\r\n%10lu KiB available.\r\n", tot_sect / 2, fre_sect / 2);
		dPrintf("%10lu KB total drive space.\r\n%10lu KiB available.\r\n", tot_sect / 2, fre_sect / 2);
		
		f_mkdir("Usonic");
		f_mkdir("Usonic/Screenshots");
		f_mkdir("Usonic/Reports");
		
		UINT ReadLen;
		res = f_open(&fil, "Usonic/wifi.fw", FA_READ);
		f_read(&fil, sl_wfx_firmware, 310352, &ReadLen);
		//res = f_write(&fil, sl_wfx_firmware2, 310352, &bytesWrote);
		f_close(&fil);
		
		strcpy(buff, "/Usonic/Screenshots");
		ScreenShotIdx = scan_files(buff) + 1;
		dPrintf("Next File: pic_%06u.jpg\r\n", ScreenShotIdx);		
	}
	else uPrintf("No card found\r\n");

		


	
	
	AvgSettings = 500;	
	f_mount(NULL, "", 0);	
	
	if (ParseIni())
	{
		dPrintf("Config OK\r\nVessel Name: %s\r\n", Coe.VesselName);	
		dPrintf("P1: %u, P2: %u, ADC: %u, Delay: %u\r\n", 
			Glass.FirstPos,
			Glass.SecondPos,
			Glass.AdcSize,
			Glass.Delay);	
		
		dPrintf("Abs Min1: %.1f, AbsMin2: %.1f\r\nBrk1: %.1f, Brk2: %.1f\r\nTX Length: %u",
			Coe.AbsMin1,
			Coe.AbsMin2,
			Coe.AbsBreak1,
			Coe.AbsBreak2,
			Coe.PackLen);
	}
	else dPrintf("Config ERROR");	
	
	LCD_setCursor(380, 130);
	dPrintf("AVG: %u", AvgSettings);
	
	LCD_update();	
	
	
	FillOutputBuffer();
	
	
	BME280_Init();
	HAL_Delay(100);
	
	TIM4->CNT = 0;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(TX_EN1_GPIO_Port, TX_EN1_Pin, 0);
	HAL_GPIO_WritePin(TX_PD1_GPIO_Port, TX_PD1_Pin, 0);
	HAL_Delay(1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	
	
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
	LL_ADC_SetMultiDMATransfer(ADC12_COMMON, LL_ADC_MULTI_REG_DMA_LIMIT_RES12_10B);

	
	sl_lwip_init();
	sl_wfx_set_antenna_config(SL_WFX_ANTENNA_DIVERSITY);
	
	
	
	
	while (true)
	{
		
		WifiTask();
		UartTask();
	
		//start once
		if (!HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin))
		{
			float32_t AbsMax; 
			uint16_t MaxPos = 0, MaxPos2;
			float32_t AdcValue;
			uint16_t Yold = 335, Ynew;
			float MaxCor, MaxCor2;	
			uint16_t StepsR1, StepsR2;
			uint16_t StepCnt = 0;

			LCD_clearBuf();

			
			
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			
			OneMeasure();
			
			arm_fir_init_f32(&Sf, FirCoe, filterCoeff, firStateF32, Glass.AdcSize*2);
			arm_fir_f32(&Sf, AdcIn, AdcIn, Glass.AdcSize*2);

			
			for (uint16_t x = 0; x < Glass.AdcSize*2; x++) 
			{
				AdcIn[x] = AdcIn[x + FirCoe / 2];
				AdcOut[x] = powf(AdcIn[x], 2);
			}
			
			arm_fir_init_f32(&Sf, FirOutCoe, FilterOut, firStateF32, Glass.AdcSize*2);
			arm_fir_f32(&Sf, AdcOut, AdcOut, Glass.AdcSize*2);
			
			AbsMax = 0;
			for (uint16_t x = (uint16_t)Coe.AbsBreak1; x < Glass.AdcSize; x++) 
			{
				AdcOut[x] = sqrtf(AdcOut[x + FirOutCoe / 2]);
				if (AdcOut[x] > AbsMax) 
				{
					AbsMax = AdcOut[x];
					MaxPos = x;
				}
				else if (AdcOut[x] < AbsMax - Coe.AbsMin2) break;

			}
						
//			//first signal
//			
//			MaxCor = 0;
//			for (uint16_t x = 0; x < 512; x++) 
//			{
//				AdcValue = AdcOut[x + Glass.FirstPos];
//				if (AdcValue > MaxCor) 
//				{
//					if (AdcValue > (AbsMax / Coe.AbsMin1))
//					{
//						MaxCor = AdcValue;
//						MaxPos = x;
//					}
//				}
//				else if (AdcValue < MaxCor - AbsMax / Coe.AbsBreak1)  
//				{
//					StepsR1 = x;	
//					break;
//				}
//			}
//			
		
			

//			
//			//second signal
//			
//			MaxCor2 = 0;
//			MaxPos2 = 0;
//			for (uint16_t x = 0; x < 512; x++) 
//			{
//				AdcValue = AdcOut[x + Glass.SecondPos];
////				if (AdcValue > MaxCor2 + Coe.AbsBreak2 / 10000) 
////				{
//					if (AdcValue > (AbsMax / Coe.AbsMin2))
//					{
//						StepCnt = 0;
//						MaxCor2 = AdcValue;
//						MaxPos2 = x;
//						break;
//					}
////				}
////				else// if (AdcValue < MaxCor - AbsMax / Coe.AbsBreak2 || StepCnt > STEP_BRK)
////				{
////					StepsR2 = x;
////					break;
////				}
//			}
//			
//			if (!MaxPos2) 
//			{
//				continue;
//			}
//

			
	
		
//			uint16_t ShiftPos = (MaxPos2 > MaxPos) ? MaxPos2 - MaxPos : 0;


			LCD_line(100, 10, 100, 40, 1); //maxpos 1
//			LCD_line(100, 180, 100, 210, 1); //maxpos 2
			
//			LCD_line(StepsR1 - MaxPos + 100, 20, StepsR1 - MaxPos + 100, 30, 1);
			//LCD_line(StepsR2 - MaxPos2 + 100, 190, StepsR2 - MaxPos2 + 100, 200, 1);
			
			for (uint16_t x = 0; x < scr_width; x++) 
			{
				LCD_Pixel(x, 100 - (AdcOut[x + MaxPos - 100] * 80 / AbsMax), 1);
				//LCD_Pixel(x - MaxPos2 + 100, 270 - (AdcOut[x + Glass.SecondPos] * 80 / MaxCor2), 1);
			}
		
			LCD_setCursor(380, 150);
			dPrintf("Peak: %u", MaxPos);			
			uPrintf("Peak: %u\r\n", MaxPos);
			
			
			Yold = 335;
			for (uint16_t x = 0; x < 536; x++) 
			{
				Ynew = 100 - (AdcIn[x + MaxPos - 100] * 40 / AbsMax);
				if (Ynew > 535) Ynew = 535;
				LCD_line(x, Yold, x, Ynew, 1);
				Yold = Ynew;
					
			}
//			
//			Yold = 335;
//			for (uint16_t x = 0; x < 536; x++) 
//			{
//				Ynew = 270 - (AdcIn[x + Glass.SecondPos + ShiftPos - 100 + MaxPos] * 40 / MaxCor2);
//				if (Ynew > 535) Ynew = 535;
//				LCD_line(x, Yold, x, Ynew, 1);
//				Yold = Ynew;
//					
//			}
			
			
			LCD_update();
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		}
			

		//start main proc
		if (!HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin))
		{
			wIcone = false;

			float32_t ResultAcc;
			
			float32_t MaxAvg;
			float32_t Result;
			uint16_t SubResult;
			
			FirstShift = 0;
			NewStart = true;
			FFtEn = false;
			LCD_clearBuf();
			LCD_setCursor(0, 18);
			dPrintf("Processing  AVG-%u", AvgSettings);
			LCD_setCursor(280, 18);
			dPrintf("File: pic_%06u.jpg", ScreenShotIdx);
			LCD_update();
			
			
			uPrintf("\r\n*** Processing  AVG-%u\r\n", AvgSettings);
			ResultAcc = 0;
			Result = 100;
			for (uint16_t i = 0; i < 100; i++)
			{
				OneMeasure();
				SubResult = (float)CoreProc();
				if (SubResult) Result = SubResult;
				
				ResultAcc += Result;
				WifiTask();
			}
			MaxAvg = ResultAcc / 100.0;
			OscX = 0;
			

			
			while (1)
			{
				uint16_t Rejects = 0;
				uint16_t FiltPoint = 0;		
				ResultAcc = 0;
				
				ButtonCnts = AvgSettings;
				while (ButtonCnts--)
				{
					
					WifiTask();
					UartTask();
					
					
					if (!HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin))
					{
						FFtEn = true;
						ButtonCnts = 0;
					}
					
					HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
					OneMeasure();
		
					//MaxAvg -= (MaxAvg - (float)CoreProc()) * 0.01;
					SubResult = CoreProc();
				
					
					if (SubResult > MaxAvg - 20 && SubResult < MaxAvg + 20)
					{
						Result = SubResult;
					}
					else
					{
						Rejects++;
						//uPrintf(">>>Rejected %u, avg: %.1f\r\n", SubResult, MaxAvg);
					}
					
					ResultAcc += Result;
					FiltPoint++;
						
					//ResultFilter[FiltPoint++] = Result;
					
					//uPrintf(">>>%.0f\r\n", Result);
				
					if (ButtonCnts == 0) //last measure 
					{
						//arm_rms_f32(ResultFilter, FiltPoint, &MaxAvg);
				
						uPrintf("\r\n");
						MaxAvg = ResultAcc / (float32_t)FiltPoint;

						if (FirstShift < 1) FirstShift = MaxAvg; //centering
						LCD_Pixel(OscX++, 168 - (MaxAvg - FirstShift) * 100, 1);
								
						if (OscX > 535) OscX = 0;
		
						uPrintf("Peak: %5.3f, Rejects: %u, T_ext: %.2f\r\n", MaxAvg, Rejects, TempExt);		
						

					
						res = f_mount(&fs, "", 1);
						if (res == FR_OK)
						{
							
							//sensor
							i = 0;
							while (BME280_ReadStatus())
							{
								i++;
								if (!i) break;
								HAL_Delay(10);
											
							}
									
							AnalogData.t = BME280_ReadTemperature();
							AnalogData.h = BME280_ReadHumidity();
							AnalogData.p = BME280_ReadPressure() / 1000.0f;
											
							LCD_DrawFilledRectangle(380, 40, 150, 80, 0);
							LCD_setCursor(380, 60);
							dPrintf("T: %3.1f C", AnalogData.t);
							LCD_setCursor(380, 80);
							dPrintf("P: %.1f hPa", AnalogData.p);
							LCD_setCursor(380, 100);
							dPrintf("H: %2.0f %%", AnalogData.h);
							LCD_DrawFilledRectangle(380, 110, 150, 60, 0);
							LCD_setCursor(380, 130);
							dPrintf("AVG: %u", AvgSettings);
			
							LCD_setCursor(380, 160);
							dPrintf("EXT: %.2f", TempExt);
							//file write
							res = f_open(&fil,
								"Usonic/Reports/Report.txt", 
								FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
							UINT bytesWrote;
							
							if (NewStart)
							{
								sprintf(buff, "\r\nNew Record\r\n");
								res = f_write(&fil, buff, strlen(buff), &bytesWrote);
								NewStart = false;
							}

							sprintf(buff,
								"Peak: %5.3f, T: %.1f, P: %.0f, H: %.0f, T ext: %.2f\r\n", 
								MaxAvg,
								AnalogData.t,
								AnalogData.p,
								AnalogData.h,
								TempExt);
							
							res = f_write(&fil, buff, strlen(buff), &bytesWrote);
							f_close(&fil);
							
							//internet
							if (!TimeToSend)
							{
								TimeToSend = true;
								sprintf(StringToSend, "/yave/dm/db.php?add&d=%.3f&t=%.2f", MaxAvg, TempExt);
								//uPrintf("Request: %s\r\n", StringToSend);
							}
							
						}				
						f_mount(NULL, "", 0);	
					
						LCD_update();
					}
			
				}
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				if (FFtEn) break;
			}
			while (!HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin))
			{
				HAL_Delay(100);
			}
			wIcone = false;
		}


		
		if (!HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin))
		{
		
			AvgSettings += 100;
			if (AvgSettings > 10000) AvgSettings = 100;
			
			LCD_DrawFilledRectangle(380, 110, 150, 20, 0);
			LCD_setCursor(380, 130);
			dPrintf("AVG: %u", AvgSettings);

			LCD_update();	
			
			
			i = 0;
			while (!HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin))
			{
				i++;
				if (i > 20)
				{
					i = 20;
					AvgSettings += 100;
					if (AvgSettings > 10000) AvgSettings = 100;
			
					LCD_DrawFilledRectangle(380, 110, 150, 30, 0);
					LCD_setCursor(380, 130);
					dPrintf("AVG: %u", AvgSettings);

					LCD_update();	
					WifiTask();
				}
				HAL_Delay(50);
			}
		}		
		
		
		
		
		
		
		
		
		
		//Analysis 
		if (!HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin))
		{
			i = 0;
			while (true)
			{
			
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
				
				//Full Scan
				FullScan();
			
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				
				if (!HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin)) 
				{
					if (i > 3) break;
					i = 0;
				}
				else 
				{
					i++;
					if (i > 50) i = 50;
				}

			}
			i = 0;
			while (!HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin))
			{
				i++;
				if (i > 30)
				{					
					
					LCD_setCursor(0, 18);
					if (WriteIni())
					{
						dPrintf("Config Saved");
					}
					else
					{
						dPrintf("Config Save Error");
					}
					LCD_update();
					while (!HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin))
					{
						HAL_Delay(100);
					}
					break;
				}
				HAL_Delay(100);
			}
			wIcone = false;
			LCD_DrawFilledRectangle(380, 110, 150, 30, 0);
			LCD_setCursor(380, 130);
			dPrintf("AVG: %u", AvgSettings);
			LCD_update();

		}
		
	
		//start PrintScreen
		if (!HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin))
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
			res = f_mount(&fs, "", 1);
			if (res == FR_OK)
			{
				//prepare buffer
				const int iWidth = scr_width, iHeight = scr_height;
				uint8_t ucMCU[64];

				
				memset(&jpg, 0, sizeof(JPEGIMAGE));
				
				jpg.iBufferSize = JPEG_FILE_BUF_SIZE;
				jpg.pOutput = jpgout;
				jpg.pHighWater = &jpgout[JPEG_FILE_BUF_SIZE - 512];
					
				rc = JPEGEncodeBegin(&jpg,
					&jpe,
					iWidth,
					iHeight, 
					JPEG_PIXEL_GRAYSCALE,
					JPEG_SUBSAMPLE_444,
					JPEG_Q_BEST);
				
				if (rc == JPEG_SUCCESS) 
				{
					memset(ucMCU, 0, sizeof(ucMCU));
				
					iMCUCount = ((iWidth + jpe.cx - 1) / jpe.cx) * ((iHeight + jpe.cy - 1) / jpe.cy);
					
					for (uint16_t yy = 0; yy < scr_height; yy += 8)
					{
						for (uint16_t xx = 0; xx < scr_width; xx += 8)
						{
							for (uint8_t my = 0; my < 8; my++)
							{
								for (uint8_t mx = 0; mx < 8; mx++)
								{
									ucMCU[mx + (my << 3)] = LCD_Point(xx + mx, yy + my) ? 0xFF : 0;
								}
							}
							rc = JPEGAddMCU(&jpg, &jpe, ucMCU, 8);
							if (rc != JPEG_SUCCESS) break;
						}
					}
					
					iDataSize = JPEGEncodeEnd(&jpg);
				}
				
				
				LCD_DrawFilledRectangle(0, 0, scr_width, 23, 0);
				LCD_setCursor(0, 18);
				dPrintf("Saved");
				LCD_setCursor(200, 18);
				dPrintf("File: pic_%06u.jpg", ScreenShotIdx);
				LCD_update();

				
				sprintf(buff, "Usonic/Screenshots/pic_%06u.jpg", ScreenShotIdx++);

				
				res = f_open(&fil, buff, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
				UINT bytesWrote;

				res = f_write(&fil, jpgout, iDataSize, &bytesWrote);
				
	
				f_close(&fil);
			}				
			f_mount(NULL, "", 0);	
			
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			
			while (!HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin))
			{
				HAL_Delay(100);
			}
		}
		
		
		
		//if something to read from FIFO
		if (!UsbReadStr)
		{
			while (!(GPIOE->IDR & GPIO_IDR_ID8))
			{
				GPIOE->BRR = GPIO_BRR_BR10; //clock read down
				ReadByte = (GPIOE->IDR) & 0xFF;
				UsbReadBuf[UsbReadPointer++] = ReadByte;
				if (ReadByte == 0x0A)
				{
					UsbReadBuf[UsbReadPointer] = 0;
					UsbReadPointer = 0;
					UsbReadStr = true;
				}
				GPIOE->BSRR = GPIO_BRR_BR10; //clock read up
			}
		}
		
		
		//USB
		if (UsbReadStr)
		{
			if (strstr(UsbReadBuf, "AlzUsonicPing"))
			{
				uPrintf("OK\r\n");
			}
			
			else if (strstr(UsbReadBuf, "AlzUsonicStart"))
			{

				//	
				//				uint16_t StartLocation = OneMeasure();
				
				AdcDone = false;
				uPrintf("adcdata,%08u", ADC_BUF_LEN);
				UsbSendDword(adc_buf, ADC_BUF_LEN);
				
			
				arm_fir_init_f32(&Sf, FirCoe, filterCoeff, firStateF32, ADC_BUF_LEN << 1);
				
				arm_fir_f32(&Sf, AdcIn, AdcOut, ADC_BUF_LEN << 1);
				
				
				LCD_clearBuf();

				uint16_t MaxPos = 0;
				//				uint16_t Glass.FirstPos = StartLocation + FirCoe / 2;
				float MaxCor, MaxOld = 0;				
				
				for (uint16_t y = 0; y < 400; y++)
				{
					MaxCor = 0;
					for (uint16_t x = 0; x < 400; x++)
					{
						MaxCor += fabs(AdcOut[x] * AdcOut[x + y]);
					}
					if (MaxCor > MaxOld)
					{
						MaxPos = y;
						MaxOld = MaxCor;
					}
					LCD_Pixel(y, 335 - (MaxCor * 1000), 1);
				}
				
				LCD_setCursor(380, 200);
				dPrintf("Peak: %u", MaxPos);
				
				uint16_t Yold = 335, Ynew;
				for (uint16_t x = 0; x < 536; x++) 
				{
					Ynew = 110 - (AdcOut[x + Glass.FirstPos] * 150);
					if (Ynew > 535) Ynew = 535;
					LCD_line(x, Yold, x, Ynew, 1);
					Yold = Ynew;
					
					//LCD_Pixel(x, 335 - (Output[x] * 10), 1);
				}
				
				//				Yold = 335;
				//				for (uint16_t x = 0; x < 536; x++) 
				//				{
				//					Ynew = 230 - (AdcOut[x + Glass.SecondPos] * 300);
				//					if (Ynew > 535) Ynew = 535;
				//					LCD_line(x, Yold, x, Ynew, 1);
				//					Yold = Ynew;
				//				}
				
				

				LCD_update();
				

				
			}
			
			UsbReadStr = false;
		}
		

		address++;
		
		if (address & 0x400000)
		{
			//ProcessStatus = MX_FATFS_Process();	
			address = 0;	
			//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			i = 0;
			while (BME280_ReadStatus())
			{
				i++;
				if (!i) break;
				HAL_Delay(10);
			
			}
	
			AnalogData.t = BME280_ReadTemperature();
			AnalogData.h = BME280_ReadHumidity();
			AnalogData.p = BME280_ReadPressure() / 1000.0f;
			
			LCD_DrawFilledRectangle(380, 40, 150, 70, 0);
			LCD_setCursor(380, 60);
			dPrintf("T: %3.1f C", AnalogData.t);
			LCD_setCursor(380, 80);
			dPrintf("P: %.1f hPa", AnalogData.p);
			LCD_setCursor(380, 100);
			dPrintf("H: %2.0f %%", AnalogData.h);
			
			LCD_DrawFilledRectangle(380, 110, 150, 60, 0);
			LCD_setCursor(380, 130);
			dPrintf("AVG: %u", AvgSettings);
			
			LCD_setCursor(380, 160);
			dPrintf("EXT: %.2f", TempExt);

			
			
			LCD_update();	
			
			
		}		
	
	}
	
	
	

	
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();

  /** MCO configuration
  */
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) 
{
	AdcDone = true;
}


//remove spaces
void stripSP(char *s) 
{
	for (char *d = s; (*d = *s) != '\0'; s++) d += !isspace(*d);
}


bool WriteIni()
{
	
	size_t Pos = 0;
	uint16_t PrintPos = 0;
	uint16_t bytesRead;
	char *tPos, *tEnd;

	res = f_mount(&fs, "", 1);
	if (res != FR_OK) return false;
	res = f_open(&fil, "Usonic/config.ini", FA_READ);
	if (res != FR_OK) return false;
	
	while (true)
	{
		f_gets(buff, 128, &fil);
		bytesRead = strlen(buff);
		if (!bytesRead) break;
		tPos = strstr(buff, "=");
		
		if (strstr(buff, "AbsMin2"))
		{
			stripSP(buff);
			PrintPos += sprintf(WifiPayload + PrintPos, "AbsMin2=%.4f\n", Coe.AbsMin2);
		}
		else if (strstr(buff, "AbsBreak2"))
		{
			stripSP(buff);
			PrintPos += sprintf(WifiPayload + PrintPos, "AbsBreak2=%.1f\n", Coe.AbsBreak2);
		}
		else if (strstr(buff, "AbsBreak1"))
		{
			stripSP(buff);
			PrintPos += sprintf(WifiPayload + PrintPos, "AbsBreak1=%.1f\n", Coe.AbsBreak1);
		}
		else
		{
			PrintPos += sprintf(WifiPayload + PrintPos, "%s", buff);
		}

	}

	f_close(&fil);
	
	//file write
	res = f_open(&fil, "Usonic/config.ini", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	if (res != FR_OK) return false;
	UINT bytesWrote;
						
	res = f_write(&fil, WifiPayload, PrintPos, &bytesWrote);
	if (res != FR_OK) return false;
	f_close(&fil);
	
	//Vessel update
	sprintf(buff, "Usonic/%s", Coe.VesselName);
	res = f_open(&fil, buff, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	if (res != FR_OK) return false;
	
	PrintPos = sprintf(WifiPayload,
		"VesParm = %u, %u, %u, %u, %u\n", 
		Glass.AdcSize,
		Glass.FirstPos+100,
		Glass.SecondPos+200,
		Glass.Delay,
		Coe.PackLen);
	
	res = f_write(&fil, WifiPayload, PrintPos, &bytesWrote);
	if (res != FR_OK) return false;
	
	f_close(&fil);
	
	f_mount(NULL, "", 0);	
	return true;
	
}


bool ParseIni()
{

	uint16_t bytesRead;
	char *tPos, *tEnd;

	res = f_mount(&fs, "", 1);
	if (res != FR_OK) return false;
	res = f_open(&fil, "Usonic/config.ini", FA_READ);
		
	//ini parse
	while (true)
	{
		f_gets(buff, 128, &fil);
		bytesRead = strlen(buff);
		stripSP(buff);
		tPos = strstr(buff, "=");
		if (strstr(buff, "AbsMin1="))
		{
			Coe.AbsMin1 = atof(tPos + 1);
		}
		else if (strstr(buff, "AbsMin2="))
		{
			Coe.AbsMin2 = atof(tPos + 1);
		}			
		else if (strstr(buff, "AbsBreak1="))
		{
			Coe.AbsBreak1 = atof(tPos + 1);
		}			
		else if (strstr(buff, "AbsBreak2="))
		{
			Coe.AbsBreak2 = atof(tPos + 1);
		}
		//		else if (strstr(buff, "PackLen="))
		//		{
		//			Coe.PackLen = atol(tPos + 1);
		//		}
		
		else if(strstr(buff, "Freq="))
		{
			Coe.Freq = atol(tPos + 1);
		}
		else if(strstr(buff, "Model="))
		{
			sprintf(Coe.VesselName, "%s.cfg", tPos + 1);
		}
		else if(strstr(buff, "WifiSsid="))
		{
			sprintf(wlan_ssid, "%s", tPos + 1);
		}
		else if(strstr(buff, "WifiPass="))
		{
			sprintf(wlan_passkey, "%s", tPos + 1);
		}
			
		if (!bytesRead) break;
	}
	f_close(&fil);
		
	//vessel parse
	sprintf(buff, "Usonic/%s", Coe.VesselName);
		
	res = f_open(&fil, buff, FA_READ);
	
	if (res == FR_OK)
	{
		while (true)
		{
			f_gets(buff, 128, &fil);
			bytesRead = strlen(buff);
			stripSP(buff);
			tPos = strstr(buff, "=");
			if (strstr(buff, "VesParm="))
			{
				Glass.AdcSize = strtol(tPos + 1, &tEnd, 10);
				Glass.FirstPos = strtof(tEnd + 1, &tEnd) - 100;
				Glass.SecondPos = strtof(tEnd + 1, &tEnd) - 200;
				Glass.Delay = strtof(tEnd + 1, &tEnd);
				Coe.PackLen = atol(tEnd + 1);
			}
			
			if (!bytesRead) break;
		}
		
		f_mount(NULL, "", 0);	
	}
	else return false;

	return true;
}

void WifiTask()
{
	

	sl_wfx_process();
	sl_lwip_process();
		
	if (WifiConnected)
	{
		if (!wIcone)
		{
			LCD_setCursor(510, scr_height - 3);
			dPrintf("W");
			LCD_update();
			wIcone = true;
		}
		if (!HostConnecting && !WifiReceived && TimeToSend) 
		{
			HostConnect(StringToSend); //send data
			WifiTimeout = HAL_GetTick();
		}
		if (HostConnecting)
		{
			if (!WifiReceived && (HAL_GetTick() - WifiTimeout) > 10000)
			{
					
				HostConnecting = false;
			}
		}
	}
		
		
	if (WifiReceived)
	{
		//uPrintf("Len: %u\r\n", WifiReceived);
		if (WifiReceived > 25) WifiPayload[25] = 0;
		WifiPayload[WifiReceived] = 0;
		//uPrintf("Len: %u %s\r\n", WifiReceived, WifiPayload);
		if (strstr(WifiPayload, "RECEIVED: OK"))
		{
			uPrintf("Receive confirmed\r\n");
		}
		WifiReceived = 0;
		TimeToSend = false;
	}
}


void UartTask()
{
	if (GetUart((uint8_t*)&UartRX[UartRxPos])) 
	{
		if (UartRX[UartRxPos] == '\r')
		{
			UartRX[UartRxPos] = 0;
			TempExt = atof(UartRX);
			if (TempExt > 100.0f || TempExt < -10.0f) TempExt = 0.0f;
			//uPrintf("ExtTemp: %.3f\r\n", TempExt);
			UartRxPos = 0;
		}
		else
		{
			UartRxPos++;
			if (UartRxPos == UART_FIFO) UartRxPos = 0;
		}

	}
}


void KeyAdj()
{
	
	static uint8_t bt[3];
	static bool bp[3];
	static bool ba[3];
	
	

	
	if (!HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin))
	{
		if (bt[0] < 100) bt[0]++;
	}
	else 
	{
		if (bt[0] > 0) bt[0]--;
	}
	
	if (!HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin))
	{
		if (bt[1] < 100) bt[1]++;
	}
	else 
	{
		if (bt[1] > 0) bt[1]--;
	}
	
	if (!HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin))
	{
		if (bt[2] < 100) bt[2]++;
	}
	else 
	{
		if (bt[2] > 0) bt[2]--;
	}
	
	for (uint8_t i = 0; i < 3; i++)
	{
		if (bt[i] > 10) bp[i] = true;
		else if (bt[i] == 0) 
		{
			bp[i] = false;
			ba[i] = false;
		}
	}
	
	
	if (bp[0] && !ba[0]) //menu
	{
		ba[0] = true;
		MenuPos++;
		if (MenuPos > 3) MenuPos = 0;
	}
	
	if (bp[1] && !ba[1]) //line up
	{
		ba[1] = true;
		if (MenuPos == 0) Coe.AbsMin2 -= .001;
		if (MenuPos == 1) Coe.AbsBreak1 += 50;
		if (MenuPos == 3) Coe.AbsBreak2 += 50;
		if (MenuPos == 2 && Coe.PackLen < 16)
		{
			Coe.PackLen++;
			FillOutputBuffer();
		}
	}
	
	if (bp[2] && !ba[2]) //line down
	{
		ba[2] = true;
		if (MenuPos == 0) Coe.AbsMin2 += .001;
		if (MenuPos == 1 && Coe.AbsBreak1 > 50) Coe.AbsBreak1 -= 50;
		if (MenuPos == 3 && Coe.AbsBreak2 > 50) Coe.AbsBreak2 -= 50;
		if (MenuPos == 2 && Coe.PackLen > 1)
		{
			Coe.PackLen--;
			FillOutputBuffer();
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
