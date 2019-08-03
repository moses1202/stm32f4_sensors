/**************************************************************************//**
 * @file	Thread_ICM40606.c
 * @version	V1.0
 * $Revision: 7 $
 * $Date: 19/07/28 13:38p $
 * @author	Moses
 * @brief
 *          Gyro/Accel sensing running on RTOS using ICM40606 sensors.
 * @note
 * Copyright (C) 2019 Unitec Co, Ltd. All rights reserved.
 *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "cmsis_os.h"

/* Private type define -------------------------------------------------------*/
typedef struct
{
	uint8_t header;
	uint16_t acc_x;
	uint16_t acc_y;
	uint16_t acc_z;
	uint16_t gyr_x;
	uint16_t gyr_y;
	uint16_t gyr_z;
	uint8_t temp;
	uint16_t tmst;
} ICM40606_SEN_DATA;

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
static uint8_t u8RxBuf[16] = {0};
static ICM40606_SEN_DATA sen_data[96];

/* Private definitions -------------------------------------------------------*/
#define HSPI2_TIMEOUT	0xFFFF
#define ICM40606_CHIPID	0x37
#define MAX_FIFO_SIZE	1024

//#define	PRINT_INT_STATUS

/* Private function prototypes -----------------------------------------------*/
void ICM40606_IO_Write(uint8_t *cmd, uint16_t size);
void ICM40606_IO_Read(uint8_t *cmd, uint16_t w_size, uint8_t *data, uint16_t r_size);

void Thread_ICM40606 (void const *arg)
{
	uint8_t cmd[8] = {0};
	uint16_t wtm_size = 0;
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	printf("Thread_ICM40606 started\n\r");
	osDelay(20);
	
	// Read WHO_AM_I
	cmd[0] = 0x75 | 0x80;
	ICM40606_IO_Read(cmd, 1, u8RxBuf, 1);
	if(u8RxBuf[0] != ICM40606_CHIPID)
	{
		printf("Chip ID %2X unmatched!\n\r", u8RxBuf[0]);
		_Error_Handler(__FILE__, __LINE__);
	}
	else
	{
		printf("ICM40606 chip ID: %2X\n\r", u8RxBuf[0]);
	}
	
	// Set INT_CONFIG, P.42, 43
	cmd[0] = 0x14;
	cmd[1] = 0x00 << 5	// INT2_MODE
			|0x00 << 4	// INT2_DRIVE_CIRCUIT
			|0x01 << 3	// INT2_POLARITY
			|0x00 << 2	// INT1_MODE
			|0x01 << 0;	// INT1_POLARITY
	ICM40606_IO_Write(cmd, 2);
	
	// Set FIFO_CONFIG, P.43
	cmd[0] = 0x16;
	cmd[1] = 0x01 << 6;	// FIFO_MODE
	ICM40606_IO_Write(cmd, 2);
	
	// Set INT_SOURCE0, P.54
	cmd[0] = 0x65;
	cmd[1] = 0x00 << 6	// FSYNC_INT1_EN
			|0x00 << 5	// PLL_RDY_INT1_EN
			|0x00 << 4	// RESET_DONE_INT1_EN
			|0x00 << 3	// DRDY_INT1_EN
			|0x01 << 2	// FIFO_THS_INT1_EN
			|0x00 << 1	// FIFO_FULL_INT1_EN
			|0x00 << 0;	// AGC_RDY_INT1_EN
	ICM40606_IO_Write(cmd, 2);
	
	// Set INTF_CONFIG0, P.48
	cmd[0] = 0x4C;
	cmd[1] = 0x00 << 6	// FIFO_COUNT_REC
			|0x01 << 5	// FIFO_COUNT_ENDIAN
			|0x01 << 4	// SENSOR_DATA_ENDIAN
			|0x03 << 0;	// UI_SIFS_CFG
	ICM40606_IO_Write(cmd, 3);
	
	// Set GYRO_CONFIG0, P.49
	cmd[0] = 0x4F;
	cmd[1] = 0x04 << 5	// GYRO_FS_SEL; 4=+/-125dps
			|0x08 << 0;	// GYRO_ODR; 7=200Hz, 8=100Hz
	ICM40606_IO_Write(cmd, 2);
	
	// Set ACCEL_CONFIG0, P.49
	cmd[0] = 0x50;
	cmd[1] = 0x02 << 5	// ACCEL_FS_SEL; 2=+/-4g
			|0x08 << 0;	// ACCEL_ODR; 7=200Hz, 8=100Hz
	ICM40606_IO_Write(cmd, 2);
	
	// Set FIFO_CONFIG1, P.53
	cmd[0] = 0x5F;
	cmd[1] = 0x01 << 3	// FIFO_TMST_FSYNC_EN
			|0x01 << 2	// FIFO_TEMP_EN
			|0x01 << 1	// FIFO_GYRO_EN
			|0x01 << 0;	// FIFO_ACCEL_EN
	ICM40606_IO_Write(cmd, 2);
	
	// Set FIFO_CONFIG2, P.53
	cmd[0] = 0x60;
	cmd[1] = 0x40 << 0;	// FIFO_WM[7:0]
	cmd[2] = 0x01 << 0;	// FIFO_WM[15:8]
	ICM40606_IO_Write(cmd, 3);
	
	// Clear FIFO
	// Set SIGNAL_PATH_RESET, P.48, 49
	cmd[0] = 0x4B;
	cmd[1] = 0x01 << 1;	// FIFO_FLUSH
	ICM40606_IO_Write(cmd, 2);
	
	// Set PWR_MGMT0, P.48
	cmd[0] = 0x4E;
	cmd[1] = 0x00 << 5	// TEMP_DIS
			|0x03 << 2	// GYRO_MODE
			|0x03 << 0;	// ACCEL_MODE
	ICM40606_IO_Write(cmd, 2);
	
	// Dummy read
	// Read FIFO_DATA, P.47
	cmd[0] = 0x30 | 0x80;
	ICM40606_IO_Read(cmd, 1, (uint8_t*)sen_data, sizeof(sen_data[0]));
	
	while(1)
	{
		// Wait for signal
		osSignalWait (ICM40606_INT_SIGNAL, osWaitForever);
		
		// Read INT_STATUS, P.46, 47
		cmd[0] = 0x2D | 0x80;
		ICM40606_IO_Read(cmd, 1, u8RxBuf, 1);
		#ifdef PRINT_INT_STATUS
		if(u8RxBuf[0] & 0x08)
		{
			printf("DATA_RDY_INT\n\r");
		}
		if(u8RxBuf[0] & 0x04)
		{
			printf("FIFO_THS_INT\n\r");
		}
		if(u8RxBuf[0] & 0x02)
		{
			printf("FIFO_FULL_INT\n\r");
		}
		#endif //PRINT_INT_STATUS
	
		// Read FIFO_COUNTH/L, P.47
		cmd[0] = 0x2E | 0x80;
		ICM40606_IO_Read(cmd, 1, u8RxBuf, 2);
		wtm_size = u8RxBuf[0] << 8 | u8RxBuf[1];
		if(wtm_size > MAX_FIFO_SIZE)
		{
			// Clear FIFO
			// Set SIGNAL_PATH_RESET, P.48, 49
			cmd[0] = 0x4B;
			cmd[1] = 0x01 << 1;	// FIFO_FLUSH
			ICM40606_IO_Write(cmd, 2);
		}
		else if(wtm_size > 0)
		{
			printf("wtm_size: %4d\n\r", wtm_size);
		
			// Read FIFO_DATA, P.47
			cmd[0] = 0x30 | 0x80;
			ICM40606_IO_Read(cmd, 1, (uint8_t*)sen_data, wtm_size);
			
//			for(uint8_t i = 0; i < wtm_size / 16; i++)
//			{
//				printf("ACC_X: %d, ", sen_data[i].acc_x);
//				printf("ACC_Y: %d, ", sen_data[i].acc_y);
//				printf("ACC_Z: %d, ", sen_data[i].acc_z);
//				printf("GYR_X: %d, ", sen_data[i].gyr_x);
//				printf("GYR_Y: %d, ", sen_data[i].gyr_y);
//				printf("GYR_Z: %d, ", sen_data[i].gyr_z);
//				printf("\n\r");
//			}
		}
		
		// Read FIFO_LOST_PKT0/1, P.56
		cmd[0] = 0x6C | 0x80;
		ICM40606_IO_Read(cmd, 1, u8RxBuf, 2);
		wtm_size = u8RxBuf[0] << 8 | u8RxBuf[1];
		if(wtm_size > 0)
		{
			printf("FIFO_LOST_PKT: %4d\n\r", wtm_size);
		}
	}
}

void ICM40606_IO_Write(uint8_t *cmd, uint16_t size)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &cmd[0], size, HSPI2_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}

void ICM40606_IO_Read(uint8_t *cmd, uint16_t w_size, uint8_t *data, uint16_t r_size)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &cmd[0], w_size, HSPI2_TIMEOUT);
	HAL_SPI_Receive(&hspi2, &data[0], r_size, HSPI2_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}

/*** (C) COPYRIGHT 2019 Unitec Co, Ltd ***/

