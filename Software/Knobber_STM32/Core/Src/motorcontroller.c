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

static Controller moco;

static inline uint16_t max(uint16_t value, uint16_t limit)
{
	if (value > limit)
		return limit;
	return value;
}

static inline float limit(float input, float min, float max)
{
	if (input > max)
		return max;
	if (input < min)
		return min;
	return input;
}

void initMotorControl()
{
	// initialize AS5600 encoder
	AS5600_init(&hi2c2, AS5600_CONF_L_HYST_OFF | AS5600_CONF_L_OUTS_AN | AS5600_CONF_L_PM_NOM | AS5600_CONF_L_PWMF_115, AS5600_CONF_H_FTH_SLOW | AS5600_CONF_H_SF_2x | AS5600_CONF_L_HYST_OFF);
	moco.mode = POSITION_MODE;
	moco.encoder_offset = 6;
	moco.arm_offset = 200;
	moco.angle_cw = (2048/7)/4;
	moco.angle_ccw = (2048/7)/4*3;
	moco.phase_offset1 = 0;			// 2048 / 7 / 3 * 0
	moco.phase_offset2 = 98;		// 2048 / 7 / 3 * 1
	moco.phase_offset3 = 195;		// 2048 / 7 / 3 * 2
	moco.power = 0;
	moco.power_limit = 127;
	moco.position = moco.arm_offset;
	moco.target = 0;
	moco.direction = 0;
	moco.meas_angle = 0;
	moco.old_angle = 0;
	moco.angle_error = 0;
	moco.Kp = 2.0f;
	moco.Ki = 0.005f;
	moco.Kd = 100.0f;
	moco.calibration = 0;
	moco.as5600_i2c_angle[0] = 0;
	moco.as5600_i2c_angle[1] = 0;
}

void update()
{
	HAL_I2C_Mem_Read_IT(&hi2c2, AS5600_I2C_ADDR, AS5600_REG_RAWANGLE, I2C_MEMADD_SIZE_8BIT, (uint8_t*)moco.as5600_i2c_angle, 2);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	__HAL_I2C_CLEAR_FLAG(I2cHandle, I2C_FLAG_RXNE);
	moco.meas_angle = (moco.as5600_i2c_angle[1] + (((uint16_t)moco.as5600_i2c_angle[0] & 0x0F) << 8))/2;
	//uint16_t angle = as5600_i2c_angle/2;

	if (moco.mode == TORQUE_MODE)
	{
		updateTorque();
	}
	else if (moco.mode == POSITION_MODE)
	{
		updatePosition();
	}
}

float proportional;
float integral;
float differential;

void updatePosition()
{
	// determine direction and power
	moco.position_error = (moco.target - moco.position);

	proportional 	 = moco.Kp * moco.position_error;
	moco.integrator	 = limit(moco.integrator + moco.Ki * moco.position_error, -100.0f, 100.0f);
	integral 		 = moco.integrator + moco.Ki * moco.position_error;
	differential 	 = moco.Kd * (moco.position_error - moco.old_position_error);

	float power = proportional + integral + differential;

	if (power > 0)
	{
		moco.direction = 0;
	}
	else
	{
		moco.direction = 1;
	}

	moco.power = max((uint16_t)abs(proportional + integral + differential), moco.power_limit);
	moco.old_position_error = moco.position_error;

	//moco.power = max(abs(moco.position_error), moco.power_limit);

	updateTorque(moco.meas_angle);
}

void updateTorque()
{
	// update global position and handle rotational overflow (2047.0 or 0.2047)
	moco.angle_error = (moco.meas_angle - moco.old_angle);

	// border setting must be chosen according the update rate and the max expected rpm
	const uint16_t border = 100;
	if (moco.old_angle < border)
	{
		// old_angle was slightly above 0
		if (moco.meas_angle > (2047-border))
		{
			// new angle was slightly below 2047
			// UNDERFLOW must have happened
			moco.position = moco.position - 2047;
		}
	}
	else if (moco.old_angle > (2048-border))
	{
		// old_angle was slightly below 2047
		if (moco.meas_angle < border)
		{
			// new angle was slightly above 0
			// OVERFLOW must have happened
			moco.position = moco.position + 2048;
		}
	}
	moco.position = moco.position + moco.angle_error;

	moco.old_angle = moco.meas_angle;

	if (moco.calibration)
	{
		if (moco.direction)
		{
			// thwo phases are switched !!
			htim1.Instance->CCR2 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.phase_offset1) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
			htim1.Instance->CCR1 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.phase_offset2) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
			htim1.Instance->CCR3 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.phase_offset3) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
		}
		else
		{
			// thwo phases are switched !!
			htim1.Instance->CCR2 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.phase_offset1) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
			htim1.Instance->CCR1 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.phase_offset2) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
			htim1.Instance->CCR3 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.phase_offset3) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
		}
	}
	else
	{
		if (moco.direction)
		{
			// thwo phases are switched !!
			htim1.Instance->CCR2 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.angle_cw + moco.phase_offset1) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
			htim1.Instance->CCR1 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.angle_cw + moco.phase_offset2) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
			htim1.Instance->CCR3 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.angle_cw + moco.phase_offset3) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
		}
		else
		{
			// thwo phases are switched !!
			htim1.Instance->CCR2 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.angle_ccw + moco.phase_offset1) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
			htim1.Instance->CCR1 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.angle_ccw + moco.phase_offset2) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
			htim1.Instance->CCR3 = ((sintab[(moco.meas_angle + moco.encoder_offset + moco.angle_ccw + moco.phase_offset3) % sintablen] * (uint16_t)moco.power) >> 7) + 1023;
		}
	}
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

void calibrateOffset(uint8_t calibration_power)
{
	moco.calibration = 1;
	uint16_t angle = 0;
	moco.power = calibration_power;

	while (AS5600_getRawAngle(&hi2c2) > 2048/7)
	{
		angle += 5;
		updatePosition(angle);
		HAL_Delay(100);
	}
	updatePosition(0);
	HAL_Delay(200);

	moco.encoder_offset = 0;
	for (uint8_t i = 0; i < (1 << 4); i++)
	{
		moco.encoder_offset += AS5600_getRawAngle(&hi2c2);
	}
	moco.power = 0;
	updatePosition(0);
	moco.encoder_offset = moco.encoder_offset >> 4;
	moco.calibration = 0;
	//position = encoder_offset;
	moco.target = 0;
}

void setMode(control_state_type mode)
{
	moco.mode = mode;
}

control_state_type getMode()
{
	return moco.mode;
}

uint8_t getPower()
{
	return moco.power;
}

void setDirection(bool direction)
{
	moco.direction = direction;
}

void setPowerLimit(uint8_t power_limit)
{
	moco.power_limit = power_limit;
}

void setPower(uint8_t power)
{
	moco.power = power;
}

void setTarget(uint32_t target)
{
	moco.target = target;
}

int32_t getPosition()
{
	return moco.position;
}
