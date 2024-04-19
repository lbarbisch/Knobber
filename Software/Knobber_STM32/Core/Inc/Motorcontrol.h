/*
 * Motorcontrol.h
 *
 *  Created on: Apr 6, 2024
 *      Author: Lukas
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#include "main.h"
#include "sintab.h"
#include <stdbool.h>

void initMotor();
void updateMotor();
void setMotorPower(uint16_t power);
uint8_t getMotorPower();
uint16_t getMotorAngle();
void setMotorAngle(uint16_t angle);
void setMotorDirection(bool direction);
void enableMotor();
void disableMotor();


#endif /* INC_MOTORCONTROL_H_ */
