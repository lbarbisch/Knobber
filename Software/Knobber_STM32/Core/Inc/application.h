/*
 * application.h
 *
 *  Created on: Mar 25, 2024
 *      Author: Lukas
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include "main.h"
#include "sintab.h"
#include "AP102_Driver.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void init();
void mainloop();
float getSupplyVoltage();
void setAngle(uint16_t angle);
void setPower(uint8_t power);


#endif /* INC_APPLICATION_H_ */
