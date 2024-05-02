/*
 * motorcontroller.c
 *
 *  Created on: Apr 28, 2024
 *      Author: Lukas
 */

#include "motorcontroller.h"

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;

static control_state_type motorcontroller_mode = TORQUE_MODE;
static uint8_t power = 0;
static uint8_t power_limit = 127;
static uint16_t old_angle = 0;
static uint16_t encoder_offset = 6;
//static const int32_t arm_offset = 160; // linke Motor
static const int32_t arm_offset = 200; // rechte Motor
static int32_t position = arm_offset;
static int32_t target = 0;
static bool direction = 0;
static bool calibration = 0;
static uint8_t as5600_angle[2] = {0};

static uint16_t angle = 0;

static const uint16_t angle_cw  = (2048/7)/4;
static const uint16_t angle_ccw = (2048/7)/4*3;

static inline uint16_t max(uint16_t value, uint16_t limit)
{
	if (value > limit)
		return limit;
	return value;
}

void update()
{
	HAL_I2C_Mem_Read_IT(&hi2c2, AS5600_I2C_ADDR, AS5600_REG_RAWANGLE, I2C_MEMADD_SIZE_8BIT, (uint8_t*)as5600_angle, 2);
}

void updatePosition(uint16_t angle_meas)
{
	// determine direction and power
	int32_t diff_pos = (target - position);
	if (diff_pos > 0)
	{
		// must rotate cw to reach target
		direction = 0;
	}
	if (diff_pos < 0)
	{
		// must rotate ccw to reach target
		direction = 1;
	}

	power = max(abs(diff_pos), power_limit);

	updateTorque(angle_meas);
}

void updateTorque(uint16_t angle_meas)
{
	// update global position and handle rotational overflow (2047->0 or 0->2047)
	int16_t diff_angle = (angle_meas - old_angle);

	// border setting must be chosen according the update rate and the max expected rpm
	const uint16_t border = 100;
	if (old_angle < border)
	{
		// old_angle was slightly above 0
		if (angle_meas > (2047-border))
		{
			// new angle was slightly below 2047
			// UNDERFLOW must have happened
			position = position - 2047;
		}
	}
	else if (old_angle > (2048-border))
	{
		// old_angle was slightly below 2047
		if (angle_meas < border)
		{
			// new angle was slightly above 0
			// OVERFLOW must have happened
			position = position + 2048;
		}
	}
	position = position + diff_angle;

	old_angle = angle_meas;

	if (calibration)
	{
		if (direction)
		{
			// thwo phases are switched !!
			htim1.Instance->CCR2 = ((sintab[(angle_meas+encoder_offset)     % sintablen] * (uint16_t)power) >> 7) + 1023;
			htim1.Instance->CCR1 = ((sintab[(angle_meas+encoder_offset+98)  % sintablen] * (uint16_t)power) >> 7) + 1023;
			htim1.Instance->CCR3 = ((sintab[(angle_meas+encoder_offset+195) % sintablen] * (uint16_t)power) >> 7) + 1023;
		}
		else
		{
			// thwo phases are switched !!
			htim1.Instance->CCR2 = ((sintab[(angle_meas+encoder_offset)     % sintablen] * (uint16_t)power) >> 7) + 1023;
			htim1.Instance->CCR1 = ((sintab[(angle_meas+encoder_offset+98)  % sintablen] * (uint16_t)power) >> 7) + 1023;
			htim1.Instance->CCR3 = ((sintab[(angle_meas+encoder_offset+195) % sintablen] * (uint16_t)power) >> 7) + 1023;
		}
	}
	else
	{
		if (direction)
		{
			// thwo phases are switched !!
			htim1.Instance->CCR2 = ((sintab[(angle_meas+encoder_offset+angle_cw)     % sintablen] * (uint16_t)power) >> 7) + 1023;
			htim1.Instance->CCR1 = ((sintab[(angle_meas+encoder_offset+angle_cw+98)  % sintablen] * (uint16_t)power) >> 7) + 1023;
			htim1.Instance->CCR3 = ((sintab[(angle_meas+encoder_offset+angle_cw+195) % sintablen] * (uint16_t)power) >> 7) + 1023;
		}
		else
		{
			// thwo phases are switched !!
			htim1.Instance->CCR2 = ((sintab[(angle_meas+encoder_offset+angle_ccw)     % sintablen] * (uint16_t)power) >> 7) + 1023;
			htim1.Instance->CCR1 = ((sintab[(angle_meas+encoder_offset+angle_ccw+98)  % sintablen] * (uint16_t)power) >> 7) + 1023;
			htim1.Instance->CCR3 = ((sintab[(angle_meas+encoder_offset+angle_ccw+195) % sintablen] * (uint16_t)power) >> 7) + 1023;
		}
	}
}

void setMode(control_state_type mode)
{
	motorcontroller_mode = mode;
}

control_state_type getMode()
{
	return motorcontroller_mode;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	__HAL_I2C_CLEAR_FLAG(I2cHandle, I2C_FLAG_RXNE);
	angle = (as5600_angle[1] + (((uint16_t)as5600_angle[0] & 0x0F) << 8))/2;
	//uint16_t angle = as5600_angle/2;

	if (motorcontroller_mode == TORQUE_MODE)
	{
		updateTorque(angle);
	}
	else if (motorcontroller_mode == POSITION_MODE)
	{
		updatePosition(angle);
	}
}

void calibrateOffset(uint8_t calibration_power)
{
	calibration = 1;
	uint16_t angle = 0;
	power = calibration_power;

	while (AS5600_getRawAngle(&hi2c2) > 2048/7)
	{
		angle += 5;
		updatePosition(angle);
		HAL_Delay(100);
	}
	updatePosition(0);
	HAL_Delay(200);

	encoder_offset = 0;
	for (uint8_t i = 0; i < (1 << 4); i++)
	{
		encoder_offset += AS5600_getRawAngle(&hi2c2);
	}
	power = 0;
	updatePosition(0);
	encoder_offset = encoder_offset >> 4;
	calibration = 0;
	//position = encoder_offset;
	target = 0;
}

uint8_t getMotorPower()
{
	return power;
}

void enableMotor()
{
	// enable all phases
	HAL_GPIO_WritePin(Enable_U_GPIO_Port, Enable_U_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Enable_V_GPIO_Port, Enable_V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Enable_W_GPIO_Port, Enable_W_Pin, GPIO_PIN_SET);

	// enable PWM generation
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	// wakeup driver chip
	HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
}

void initMotorControl()
{
	// initialize AS5600 encoder
	AS5600_init(&hi2c2, AS5600_CONF_L_HYST_OFF | AS5600_CONF_L_OUTS_AN | AS5600_CONF_L_PM_NOM | AS5600_CONF_L_PWMF_115, AS5600_CONF_H_FTH_SLOW | AS5600_CONF_H_SF_2x | AS5600_CONF_L_HYST_OFF);
}

void setDirection(bool _direction)
{
	direction = _direction;
}

void setPowerLimit(uint8_t _power_limit)
{
	power_limit = _power_limit;
}

void setPower(uint8_t _power)
{
	power = _power;
}

void setTarget(uint32_t _target)
{
	target = _target;
}

int32_t getMotorPos()
{
	return position;
}
