/*
 * AS5600.h
 *
 *  Created on: Apr 5, 2024
 *      Author: Lukas
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "main.h"

// slave addresses
static const uint8_t AS5600L_I2C_ADDR 		= 0x80;
static const uint8_t AS5600_I2C_ADDR 		= 0x6C;

// registers
static const uint8_t AS5600_REG_CONF_H 	= 0x07;
static const uint8_t AS5600_REG_CONF_L		= 0x08;
static const uint8_t AS5600_REG_RAWANGLE_H = 0x0C;
static const uint8_t AS5600_REG_RAWANGLE_L = 0x0D;
static const uint8_t AS5600_REG_ANGLE_H	= 0x0E;
static const uint8_t AS5600_REG_ANGLE_L	= 0x0F;
static const uint8_t AS5600_REG_STATUS		= 0x0B;
static const uint8_t AS5600_REG_AGC		= 0x1A;
static const uint8_t AS5600_REG_MAG_H		= 0x1B;
static const uint8_t AS5600_REG_MAG_L		= 0x1C;

// values for registers
static const uint8_t AS5600_CONF_H_WD			= 1 << 5;
static const uint8_t AS5600_CONF_H_FTH_SLOW	= 0 << 2;
static const uint8_t AS5600_CONF_H_FTH_6LSB	= 1 << 2;
static const uint8_t AS5600_CONF_H_FTH_7LSB	= 2 << 2;
static const uint8_t AS5600_CONF_H_FTH_9LSB	= 3 << 2;
static const uint8_t AS5600_CONF_H_FTH_10LSB	= 7 << 2;
static const uint8_t AS5600_CONF_H_FTH_18LSB	= 4 << 2;
static const uint8_t AS5600_CONF_H_FTH_21LSB	= 5 << 2;
static const uint8_t AS5600_CONF_H_FTH_24LSB	= 6 << 2;
static const uint8_t AS5600_CONF_H_SF_16x		= 0;
static const uint8_t AS5600_CONF_H_SF_8x		= 1;
static const uint8_t AS5600_CONF_H_SF_4x		= 2;
static const uint8_t AS5600_CONF_H_SF_2x		= 3;
static const uint8_t AS5600_CONF_L_PWMF_115	= 0 << 6;
static const uint8_t AS5600_CONF_L_PWMF_230	= 1 << 6;
static const uint8_t AS5600_CONF_L_PWMF_460	= 2 << 6;
static const uint8_t AS5600_CONF_L_PWMF_920	= 3 << 6;
static const uint8_t AS5600_CONF_L_OUTS_AN		= 0 << 4;
static const uint8_t AS5600_CONF_L_OUTS_ANR	= 1 << 4;
static const uint8_t AS5600_CONF_L_OUTS_PWM	= 2 << 4;
static const uint8_t AS5600_CONF_L_HYST_OFF	= 0 << 2;
static const uint8_t AS5600_CONF_L_HYST_1LSB	= 1 << 2;
static const uint8_t AS5600_CONF_L_HYST_2LSB	= 2 << 2;
static const uint8_t AS5600_CONF_L_HYST_3LSB	= 3 << 2;
static const uint8_t AS5600_CONF_L_PM_NOM		= 0;
static const uint8_t AS5600_CONF_L_PM_LPM1		= 1;
static const uint8_t AS5600_CONF_L_PM_LPM2		= 2;
static const uint8_t AS5600_CONF_L_PM_LPM3		= 3;
static const uint8_t AS5600_STATUS_MD			= 1 << 5;
static const uint8_t AS5600_STATUS_ML			= 1 << 4;
static const uint8_t AS5600_STATUS_MH			= 1 << 3;

void AS5600_init(I2C_HandleTypeDef hi2c, uint8_t conf_L, uint8_t conf_H);
uint16_t AS5600_getAngle(I2C_HandleTypeDef hi2c);
uint16_t AS5600_getRawAngle(I2C_HandleTypeDef hi2c);
uint8_t AS5600_getStatus(I2C_HandleTypeDef hi2c);
uint16_t AS5600_getMagnitude(I2C_HandleTypeDef hi2c);


#endif /* INC_AS5600_H_ */
