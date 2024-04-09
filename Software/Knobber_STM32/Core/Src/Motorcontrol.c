/*
 * Motorcontrol.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Lukas
 */

#include "Motorcontrol.h"

uint8_t _power = 0;
uint16_t _angle = 0;
bool _direction = false;
static const uint16_t _angle_cw = 73;
static const uint16_t _angle_ccw = 219;

extern TIM_HandleTypeDef htim1;

void initMotor()
{
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	// wakeup driver chip
	HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
}

void updateMotor()
{
	if (_direction)
	{
		// thwo phases are switched !!
		htim1.Instance->CCR2 = ((sintab[(_angle+_angle_cw)     % sintablen] * (uint16_t)_power) >> 7) + 1023;
		htim1.Instance->CCR1 = ((sintab[(_angle+_angle_cw+98)  % sintablen] * (uint16_t)_power) >> 7) + 1023;
		htim1.Instance->CCR3 = ((sintab[(_angle+_angle_cw+195) % sintablen] * (uint16_t)_power) >> 7) + 1023;
	}
	else
	{
		// thwo phases are switched !!
		htim1.Instance->CCR2 = ((sintab[(_angle+_angle_ccw)     % sintablen] * (uint16_t)_power) >> 7) + 1023;
		htim1.Instance->CCR1 = ((sintab[(_angle+_angle_ccw+98)  % sintablen] * (uint16_t)_power) >> 7) + 1023;
		htim1.Instance->CCR3 = ((sintab[(_angle+_angle_ccw+195) % sintablen] * (uint16_t)_power) >> 7) + 1023;
	}
}

void setMotorPower(uint8_t power)
{
	if (power > 127) power = 127;
	_power = power;
}

uint16_t getMotorAngle()
{
	return _angle;
}

void setMotorAngle(uint16_t angle)
{
	_angle = angle;
}

void setMotorDirection(bool direction)
{
	_direction = direction;
}

void enableMotor()
{
	HAL_GPIO_WritePin(Enable_U_GPIO_Port, Enable_U_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Enable_V_GPIO_Port, Enable_V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Enable_W_GPIO_Port, Enable_W_Pin, GPIO_PIN_SET);
}

void disableMotor()
{
	HAL_GPIO_WritePin(Enable_U_GPIO_Port, Enable_U_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Enable_V_GPIO_Port, Enable_V_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Enable_W_GPIO_Port, Enable_W_Pin, GPIO_PIN_RESET);
}
