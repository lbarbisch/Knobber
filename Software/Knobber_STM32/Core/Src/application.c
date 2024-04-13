/*
 * application.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Lukas
 */

#include "application.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;

extern uint8_t _angle;

volatile uint16_t currentAngle = 0;
bool faultstate = false;
uint8_t status = 0;
float voltage = 0;

uint16_t adc_dma_results[5];

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
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_results, 5);

	setMotorPower(32);
}


void mainloop()
{
	//angle = AS5600_getRawAngle(hi2c2);

	currentAngle = getMotorAngle();

	if ((attractor(1024-128, 32) + attractor(1024+128, 32)) == 0)
	{
		setMotorPower(0);
	}
	/*
	const uint16_t deadzone = 10;
	uint16_t localPower = abs((int16_t)currentAngle-1024)-deadzone;

	//localPower = 32;


	if (localPower < 0)							localPower = 0;
	else if (localPower > 127)					localPower = 127;

	if (currentAngle > (1024+deadzone)) 		setMotorDirection(true);
	else if (currentAngle < (1024-deadzone)) 	setMotorDirection(false);
	else 										localPower = 0;

	setMotorPower(localPower);
	*/
}


uint8_t attractor(uint16_t position, uint16_t range)
{
	uint16_t currentAngle = getMotorAngle();
	// check if currentAngle is within range
	if ((currentAngle < (position + range)) & (currentAngle > (position - range)))
	{
		setMotorPower(abs(currentAngle - position)*4);
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
}


float getSupplyVoltage()
{
	voltage = adc_dma_results[3] / 125.27;
	return voltage;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	faultstate = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);

	setMotorAngle((adc_dma_results[4]/2)*0.25 + getMotorAngle()*0.75);
	updateMotor();
}
