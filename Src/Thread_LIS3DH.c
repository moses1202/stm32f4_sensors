/**************************************************************************//**
 * @file	Thread_LIS3DH.c
 * @version	V1.0
 * $Revision: 7 $
 * $Date: 19/07/13 06:19p $
 * @author	Moses
 * @brief
 *          Accelerometer sensing running on RTOS using LIS3DH sensors.
 * @note
 * Copyright (C) 2019 Unitec Co, Ltd. All rights reserved.
 *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
static uint8_t u8RxBuf[1024] = {0};
uint32_t	g_data_rate;

/* Private definitions -------------------------------------------------------*/
#define HSPI1_TIMEOUT	0xFFFF
#define LIS3DH_CHIPID	0x3F

#define FIFO_ENABLED
#define PRINT_DATA

/* Private function prototypes -----------------------------------------------*/
void LIS3DH_IO_Write(uint8_t *cmd, uint16_t size);
void LIS3DH_IO_Read(uint8_t *cmd, uint16_t w_size, uint8_t *data, uint16_t r_size);
uint32_t GetDataCountAndReset(void);

void Thread_LIS3DH (void const *arg)
{
	//osStatus status;
	uint8_t cmd[8] = {0};
	int16_t out[5];
	int8_t offset[3];
#ifdef FIFO_ENABLED
	uint8_t wtm_lvl;
#endif //FIFO_ENABLED
	
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	printf("Thread_LIS3DH started\n\r");
	osDelay(20);
	
	// Init
	g_data_rate = 0;
	
	cmd[0] = 0x23;
	cmd[1] = 0x01;	// STRT
	LIS3DH_IO_Write(cmd, 2);
	
	// Read info
	cmd[0] = 0x0D | 0x80;
	LIS3DH_IO_Read(cmd, 1, u8RxBuf, 3);
	printf("INFO1: 		%02X\n\r", u8RxBuf[0]);
	printf("INFO2: 		%02X\n\r", u8RxBuf[1]);
	printf("WHO_AM_I:	%02X\n\r", u8RxBuf[2]);
	
	if(u8RxBuf[2] != LIS3DH_CHIPID)
	{
		printf("Chip ID unmatched!\n\r");
		_Error_Handler(__FILE__, __LINE__);
	}
	
	// Set CTRL_REG4, P.38, 39
	cmd[0] = 0x20;
	cmd[1] = 0x06 << 4	// ODR	4=25Hz, 5=50Hz, 6=100Hz, 7=400Hz, 8=800Hz
			|0x07 << 0;	// BDU/Xen/Yen/Zen
	LIS3DH_IO_Write(cmd, 2);
	
	// Set CTRL_REG3, P.40
	cmd[0] = 0x23;
	#ifndef FIFO_ENABLED
	cmd[1] = 0x01 << 7	// DR_EN
			|0x00 << 6	// IEA
			|0x00 << 5	// IEL
			|0x00 << 4	// INT2_EN
			|0x01 << 3	// INT1_EN
			|0x00 << 2	// VFILT
			|0x00 << 0;	// STRT
	#else
	cmd[1] = 0x00 << 7	// DR_EN
			|0x00 << 6	// IEA
			|0x00 << 5	// IEL
			|0x00 << 4	// INT2_EN
			|0x01 << 3	// INT1_EN
			|0x00 << 2	// VFILT
			|0x00 << 0;	// STRT
	#endif //FIFO_ENABLED
	LIS3DH_IO_Write(cmd, 2);
	
//	// Set CTRL_REG5, P.41
//	cmd[0] = 0x24;
//	cmd[1] = 0x00 << 6	// BW, anti-aliasing bandwidth
//			|0x00 << 4	// FSCALE, full-scale selection
//			|0x00 << 2	// ST, self-test
//			|0x00 << 0;	// SIM, SPI mode
//	LIS3DH_IO_Write(cmd, 2);
	
	#ifdef FIFO_ENABLED
	// Set CTRL_REG6, P.41, 42
	cmd[0] = 0x25;
	cmd[1] = 0x00 << 7	// BOOT
			|0x01 << 6	// FIFO_EN
			|0x01 << 5	// WTM_EN
			|0x01 << 4	// ADD_INC	/* SHOULD ENABLE FOR READING FIFO DATA */
			|0x00 << 3	// P1_EMPTY
			|0x01 << 2	// P1_WTM
			|0x00 << 1	// P1_OVERRUN
			|0x00 << 0;	// P2_BOOT
	LIS3DH_IO_Write(cmd, 2);
	
	// Set FIFO_CTRL, P.44
	cmd[0] = 0x2E;
	cmd[1] = 0x02 << 5	// FMODE, FIFO mode selection
			|0x1F << 0;	// WTMP, FIFO watermark pointer
	LIS3DH_IO_Write(cmd, 2);
	#endif //FIFO_ENABLED
	
	while(1)
	{
		// Wait for signal
		osSignalWait (LIS3DH_INT_SIGNAL, osWaitForever);
		
	#ifndef FIFO_ENABLED
		// Check status, P.35
		cmd[0] = 0x18 | 0x80;
		LIS3DH_IO_Read(cmd, 1, u8RxBuf, 1);
		if(u8RxBuf[0] & 0x01)
		{
			g_data_rate++;
		}
		else if(u8RxBuf[0] & 0x02)
		{
			printf("ERROR: Data overrun!!!\n\r");
			continue;
		}
		else
		{
			// If data not ready then continue
			printf("Data not ready, STAT: %02X\n\r", u8RxBuf[0]);
			continue;
		}
		
		// Get OFF_X/Y/Z
		cmd[0] = 0x10 | 0x80;
		LIS3DH_IO_Read(cmd, 1, u8RxBuf, 3);
		memcpy(offset, u8RxBuf, 3);
	
		// Get OUT_X/Y/Z
		cmd[0] = 0x28 | 0x80;
		LIS3DH_IO_Read(cmd, 1, u8RxBuf, 6);
		memcpy(out, u8RxBuf, 6);
		
		#ifdef PRINT_DATA
		printf("OUT_X: %+0.5d, ",	out[0] - offset[0]);
		printf("OUT_Y: %+0.5d, ",	out[1] - offset[1]);
		printf("OUT_Z: %+0.5d\n\r",	out[2] - offset[2]);
		#endif //PRINT_DATA
	#else
		// Check FIFO status
		cmd[0] = 0x2F | 0x80;
		LIS3DH_IO_Read(cmd, 1, u8RxBuf, 1);
		wtm_lvl = u8RxBuf[0] & 0x1F;
		
		// If FIFO reach watermark
		if(u8RxBuf[0] & 0x80)
		{
			//printf("WTM_LVL: %d\n\r", wtm_lvl);
			
//			if(u8RxBuf[0] & 0x40)
//			{
//				printf("ERROR: Data overrun!!!\n\r");
//			}
		}
		else
		{
			printf("WTM_LVL: %d, FIFO not full yet\n\r", wtm_lvl);
			continue;
		}
		
		// Get OFF_X/Y/Z
		cmd[0] = 0x10 | 0x80;
		LIS3DH_IO_Read(cmd, 1, u8RxBuf, 3);
		memcpy(offset, u8RxBuf, 3);
	
		// Get OUT_X/Y/Z
		cmd[0] = 0x28 | 0x80;
		LIS3DH_IO_Read(cmd, 1, u8RxBuf, 6 * wtm_lvl);
		
		for(uint16_t i = 0, idx = 0; i < wtm_lvl; i++, idx += 6)
		{
			memcpy(out, &u8RxBuf[idx], 6);
			g_data_rate++;
		
		#ifdef PRINT_DATA
			printf("OUT_X: %+0.5d, ",	out[0] - offset[0]);
			printf("OUT_Y: %+0.5d, ",	out[1] - offset[1]);
			printf("OUT_Z: %+0.5d\n\r",	out[2] - offset[2]);
		#endif //PRINT_DATA
		}
	#endif //FIFO_ENABLED
	}
}

void LIS3DH_IO_Write(uint8_t *cmd, uint16_t size)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd[0], size, HSPI1_TIMEOUT);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void LIS3DH_IO_Read(uint8_t *cmd, uint16_t w_size, uint8_t *data, uint16_t r_size)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd[0], w_size, HSPI1_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &data[0], r_size, HSPI1_TIMEOUT);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

uint32_t GetDataCountAndReset(void)
{
	uint32_t data_count = g_data_rate;
	g_data_rate = 0;
	return data_count;
}

/*** (C) COPYRIGHT 2019 Unitec Co, Ltd ***/

