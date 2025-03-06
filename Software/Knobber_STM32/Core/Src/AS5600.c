/*
 * AS5600.c
 *
 *  Created on: Apr 5, 2024
 *      Author: Lukas
 */

#include "AS5600.h"


void AS5600_init(I2C_HandleTypeDef *hi2c, uint8_t conf_L, uint8_t conf_H)
{
	HAL_I2C_Mem_Write(hi2c, AS5600_I2C_ADDR, AS5600_REG_CONF_L, I2C_MEMADD_SIZE_8BIT, &conf_L, 1, 100);
	HAL_I2C_Mem_Write(hi2c, AS5600_I2C_ADDR, AS5600_REG_CONF_H, I2C_MEMADD_SIZE_8BIT, &conf_H, 1, 100);
}

uint16_t AS5600_getAngle(I2C_HandleTypeDef *hi2c)
{
	uint8_t _as5600_angle[2] = {0};
	HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&_as5600_angle, 2, 100);
	return _as5600_angle[1] + (((uint16_t)_as5600_angle[0] & 0x0F) << 8);
}

uint16_t AS5600_getRawAngle(I2C_HandleTypeDef *hi2c)
{
	uint8_t _as5600_angle[2] = {0};
	HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_REG_RAWANGLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&_as5600_angle, 2, 100);
	return _as5600_angle[1] + (((uint16_t)_as5600_angle[0] & 0x0F) << 8);
}

uint8_t AS5600_getStatus(I2C_HandleTypeDef *hi2c)
{
	uint8_t _status = 0;
	HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_REG_STATUS, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&_status, 1, 100);
	return _status & 0b00111000;
}

uint16_t AS5600_getMagnitude(I2C_HandleTypeDef *hi2c)
{
	uint8_t _magnitude[2] = {0};
	HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_REG_MAG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&_magnitude, 2, 100);
	return _magnitude[1] + (((uint16_t)_magnitude[0] & 0x0F) << 8);
}
