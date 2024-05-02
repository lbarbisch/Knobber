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

typedef struct
{
	// state of the motor controller
	control_state_type mode;// = TORQUE_MODE;

	// correction values and constants
	uint16_t encoder_offset;
	//const int32_t arm_offset = 160; // linke Motor
	int32_t arm_offset; // rechte Motor
	uint16_t angle_cw;
	uint16_t angle_ccw;
	uint16_t phase_offset1;
	uint16_t phase_offset2;
	uint16_t phase_offset3;

	// process parameters
	uint8_t power;
	uint8_t power_limit;
	int32_t position;
	int32_t position_error;
	int32_t target;
	bool direction;
	uint16_t meas_angle;
	uint16_t old_angle;
	int16_t angle_error;

	// control loop parameters
	float Kp;
	float Ki;
	float Kd;
	float integrator;
	int16_t old_position_error;

	bool calibration;
	uint8_t as5600_i2c_angle[2];

} Controller;


void initMotorControl();
void update();
void updatePosition();
void updateTorque();
void enableMotor();
void calibrateOffset(uint8_t power);
void setMode(control_state_type mode);
control_state_type getMode();
void setDirection(bool _direction);
void setPower(uint8_t _power);
uint8_t getPower();
void setPowerLimit(uint8_t _power_limit);
void setTarget(uint32_t _target);
int32_t getPosition();

#endif /* INC_MOTORCONTROLLER_H_ */
