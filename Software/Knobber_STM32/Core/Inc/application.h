/*
 * application.h
 *
 *  Created on: Mar 25, 2024
 *      Author: Lukas
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include "main.h"
#include "AP102_Driver.h"
#include "AS5600.h"
#include "Motorcontrol.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void init();
void mainloop();
float getSupplyVoltage();
void continuousIndents(uint8_t numIndents);
uint8_t attractor(uint16_t position, uint16_t range);


#endif /* INC_APPLICATION_H_ */
