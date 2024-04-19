/*
 * application.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Lukas
 */

#include "application.h"
#include "stm32g0xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;

extern uint16_t _angle;

volatile uint16_t currentAngle = 0;
volatile uint8_t Indents = 16;
bool faultstate = false;
uint8_t status = 0;
float voltage = 0;

uint16_t dma_results[5];

uint8_t Buffer[25] = {0};

const uint16_t angleoffset = 268;

volatile bool dir = false;


void init()
{
	// setup AS5600
	AS5600_init(hi2c2, AS5600_CONF_L_HYST_OFF | AS5600_CONF_L_OUTS_AN | AS5600_CONF_L_PM_NOM | AS5600_CONF_L_PWMF_115,
					   AS5600_CONF_H_FTH_SLOW | AS5600_CONF_H_SF_2x | AS5600_CONF_L_HYST_OFF
					   );

	setColor(0, 0, 128, 32);

	initMotor();
	enableMotor();

	// start ADC DMA for current, supply voltage and Motorangle
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) dma_results, 5);
	//HAL_I2C_Mem_Read_IT(&hi2c2, AS5600_I2C_ADDR, AS5600_REG_RAWANGLE_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *) dma_results, 2);

	//setMotorPower(32);
}


void mainloop()
{
	//setMotorAngle(AS5600_getRawAngle(hi2c2)/2*0.25 + getMotorAngle()*0.75);
	setMotorAngle(AS5600_getRawAngle(hi2c2)/2);

	currentAngle = getMotorAngle();

	continuousIndents(Indents);

	updateMotor();
}

void continuousIndents(uint8_t numIndents)
{
	const uint16_t rasterangle = 2048/numIndents;
	uint8_t actives = 0;
	for (uint16_t i = 0; i < 2048/rasterangle; i++)
	{
		actives += attractor(rasterangle/2+(i)*rasterangle, rasterangle/4);
	}

	if (actives == 0) setMotorPower(0);
}

uint8_t attractor(uint16_t position, uint16_t range)
{
	uint16_t currentAngle = getMotorAngle();
	// check if currentAngle is within range
	if ((currentAngle < (position + range)) & (currentAngle > (position - range)) & (currentAngle != 2047))
	{
		if (range > 10)
		{
			setMotorPower((uint8_t)abs(currentAngle - position)*(127.0f/range+1));
		}
		else
		{
			setMotorPower(abs(currentAngle - position)*(127/range+1)/2);
		}
		if (currentAngle > position)
		{
			setMotorDirection(true);
		}
		else if (currentAngle < position)
		{
			setMotorDirection(false);
		}
		return 1;
	}
	return 0;
}


float getSupplyVoltage()
{
	voltage = dma_results[3] / 125.27;
	return voltage;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	faultstate = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);

	//uint16_t raw_angle = (dma_results[4]/2);

	//setMotorAngle(raw_angle*0.25 + getMotorAngle()*0.75);
	//updateMotor();
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	faultstate = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);

	uint16_t raw_angle = (dma_results[0] + ((dma_results[1] & 0x0F)<<8))/2;
	//uint16_t raw_angle = (adc_dma_results[4]/2);

	setMotorAngle(raw_angle*0.25 + getMotorAngle()*0.75);
	updateMotor();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */

}

