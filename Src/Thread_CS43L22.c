/**************************************************************************//**
 * @file	Thread_CS43L22.c
 * @version	V1.0
 * $Revision: 7 $
 * $Date: 19/07/13 06:29p $
 * @author	Moses
 * @brief
 *          Audio playback running on RTOS using CS43L22 DAC.
 * @note
 * Copyright (C) 2019 Unitec Co, Ltd. All rights reserved.
 *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cs43l22_commands.h"
#include "cmsis_os.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s3;

#define CS43L22_SLAVE_READ	0x95
#define CS43L22_SLAVE_WRITE	0x94

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef I2C1_Fetch(uint8_t cmd_addr, uint8_t *r_data, uint8_t r_size);
HAL_StatusTypeDef I2C1_Write(uint8_t cmd_addr, uint8_t *w_data, uint8_t w_size);
void CS34L22_ChipOn(void);
void CS34L22_ChipOff(void);

void Thread_CS43L22 (void const *arg)
{
	HAL_StatusTypeDef ret_status = HAL_OK;
	uint8_t data[16] = {0};
	uint8_t cmd[16] = {0};
	
	// Pull up reset pin
	CS34L22_ChipOn();
	
	// Get chip ID
	ret_status = I2C1_Fetch(CS43L22_REG_GET_ID, data, 1);
	if(ret_status != HAL_OK)
	{
		printf("I2C1_Fetch: Error code %02X\n\r", ret_status);
		_Error_Handler(__FILE__, __LINE__);
	}
	printf("Chip ID: %02X, Chip Rev.: %02X\n\r", data[0] >> 3, data[0] & 0x07);
	
	// Turn on the chip
	cmd[0] = CS43L22_PWR1_UP;
	ret_status = I2C1_Write(CS43L22_REG_PWR_CTL1, cmd, 1);
	
	// Turn on headphone control
	cmd[0] = CS43L22_PWR2_HDA_ON;	// Headphone always ON
	ret_status = I2C1_Write(CS43L22_REG_PWR_CTL2, cmd, 1);
	
	// Set clocking control
	cmd[0] = CS43L22_CLK_AUTO_ON |	// Auto detect on
			CS43L22_CLK_SPD_SSM |	// Single-speed mode
			CS43L22_CLK_32K_OFF |	// 32kHz Sample Rate Group off
			CS43L22_CLK_VC_OFF |	// 27 MHz Video Clock off
			CS43L22_CLK_MLR_64 |	// Internal MCLK/LRCK Ratio 128
			CS43L22_CLK_MCDIV_OFF;	// MCLK no divide
	ret_status = I2C1_Write(CS43L22_REG_CLOCK_CTL, cmd, 1);
	
	// Set interface control 1
	cmd[0] = CS43L22_IC1_SLAVE |	// Slave mode
			CS43L22_IC1_SCPOL_OFF |	// SCLK polarity not inverted
			CS43L22_IC1_SCPOL_OFF |	// DSP mode off
			CS43L22_IC1_DIF_LJ |	// Left justified
			CS43L22_IC1_AWL_24;		// Audio word length 24bits
	ret_status = I2C1_Write(CS43L22_REG_INT_CTL1, cmd, 1);
	
	// Turn on green led
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	//printf("\n\r*** CS43L22_Init succcess!!! ***\n\r\n\r");
	
	while(1)
	{
		// TODO
	}
}

HAL_StatusTypeDef I2C1_Fetch(uint8_t cmd_addr, uint8_t *r_data, uint8_t r_size)
{
	HAL_StatusTypeDef ret_status = HAL_OK;
	uint8_t w_size = 1;
	
	ret_status = HAL_I2C_Master_Transmit(&hi2c1, CS43L22_SLAVE_WRITE, &cmd_addr, w_size, 0xFFFF);
	if(ret_status != HAL_OK)
	{
		printf("HAL_I2C_Master_Transmit: Register address %02X, Error code %02X\n\r", cmd_addr, ret_status);
		_Error_Handler(__FILE__, __LINE__);
	}
	else
	{
		ret_status = HAL_I2C_Master_Receive(&hi2c1, CS43L22_SLAVE_READ, &r_data[0], r_size, 0xFFFF);
		if(ret_status != HAL_OK)
		{
			printf("HAL_I2C_Master_Receive: Register address %02X, Error code %02X\n\r", cmd_addr, ret_status);
			_Error_Handler(__FILE__, __LINE__);
		}
	}
	
	return ret_status;
}

HAL_StatusTypeDef I2C1_Write(uint8_t cmd_addr, uint8_t *w_data, uint8_t w_size)
{
	HAL_StatusTypeDef ret_status = HAL_OK;
	
	ret_status = HAL_I2C_Master_Transmit(&hi2c1, CS43L22_SLAVE_WRITE, &cmd_addr, w_size + 1, 0xFFFF);
	if(ret_status != HAL_OK)
	{
		printf("HAL_I2C_Master_Transmit: Register address %02X, Error code %02X\n\r", cmd_addr, ret_status);
		_Error_Handler(__FILE__, __LINE__);
	}
	
	return ret_status;
}

void CS34L22_ChipOn(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
}

void CS34L22_ChipOff(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
}

/*** (C) COPYRIGHT 2019 Unitec Co, Ltd ***/

