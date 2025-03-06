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
#include "motorcontroller.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

void init();
void mainloop();
void getPhaseCurrents(float* _current);
float getSupplyVoltage();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1);
void parseCommandandUpdate(uint8_t* buffer, uint8_t index);


#endif /* INC_APPLICATION_H_ */
