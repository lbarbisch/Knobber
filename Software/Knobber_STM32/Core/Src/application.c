/*
 * application.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Lukas
 */

#include "application.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

uint8_t power = 0;
uint16_t angle = 0;
float voltage = 0;

uint16_t adc_dma_results[5];



void init()
{
	// set PWM values to 0
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	setPower(64);
	HAL_ADC_Start_DMA(&hadc1, adc_dma_results, 5);
}


void mainloop()
{
	voltage = getSupplyVoltage();
	for(uint16_t i = 64; i < 128; i++)
	{
		//setAngle(i);
		//setPower(i);
		HAL_Delay(1000);
	}
}

float getSupplyVoltage()
{
	voltage = adc_dma_results[3] / 1241.2;
	return voltage;
}


void setAngle(uint16_t anglesetting)
// angle is tenths of a degree
{
	if(anglesetting > sintablen)
	{
		anglesetting = sintablen;
	}

	htim1.Instance->CCR1 = ((sintab[(anglesetting)      % sintablen] * power) >> 7) + 1023;
	htim1.Instance->CCR2 = ((sintab[(anglesetting+1200) % sintablen] * power) >> 7) + 1023;
	htim1.Instance->CCR3 = ((sintab[(anglesetting+2400) % sintablen] * power) >> 7) + 1023;
}

void setPower(uint8_t powersetting)
{
	if(powersetting > 127)
	{
		powersetting = 127;
	}
	power = powersetting;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// range fitting and offset correction
	//TODO: save other values somewhere
	setAngle((uint16_t)adc_dma_results[4] * 0.87890625);
	HAL_GPIO_WritePin(Enable_U_GPIO_Port, Enable_U_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Enable_U_GPIO_Port, Enable_U_Pin, GPIO_PIN_RESET);
}
