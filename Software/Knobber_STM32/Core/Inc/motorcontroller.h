/*
 * motorcontroller.h
 *
 *  Created on: Apr 28, 2024
 *      Author: Lukas
 */

#ifndef INC_MOTORCONTROLLER_H_
#define INC_MOTORCONTROLLER_H_

#include "main.h"
#include "AS5600.h"
#include "sintab.h"
#include <stdbool.h>
#include <stdlib.h>

typedef enum
{
	TORQUE_MODE,
	POSITION_MODE
} control_state_type;



void update();
void updatePosition(uint16_t angle_meas);
void updateTorque(uint16_t angle_meas);
void setMode(control_state_type mode);
control_state_type getMode();
void calibrateOffset(uint8_t power);
uint8_t getMotorPower();
void enableMotor();
void initMotorControl();
void setDirection(bool _direction);
void setPower(uint8_t _power);
void setPowerLimit(uint8_t _power_limit);
void setTarget(uint32_t _target);
int32_t getMotorPos();

#endif /* INC_MOTORCONTROLLER_H_ */
