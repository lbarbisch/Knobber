/*
 * application.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Lukas
 */

#include "application.h"
#include "stm32g0xx_hal.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;

float voltage = 0;
extern Controller moco;

uint8_t rxbuf[64] = {0};
uint8_t rxchar;
uint8_t rxbuf_index = 0;

uint16_t adc_dma_results[5];
uint16_t adc_curr_idle[3];
float current[3];

uint8_t task = 0;


void init()
{
	setMode(POSITION_MODE);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_results, 5);

	HAL_Delay(50);
	adc_curr_idle[0] = adc_dma_results[0];
	adc_curr_idle[1] = adc_dma_results[1];
	adc_curr_idle[2] = adc_dma_results[2];


	enableMotor();

	initMotorControl();
	//calibrateOffset(64);
}

uint32_t timer = 0;

void mainloop()
{
	update();
	HAL_UART_Receive_IT(&huart1, &rxchar, 1);
	if (task == 1)
	{
		calibrateOffset(127);
		task = 0;
	}
	getPhaseCurrents(current);

	//char txbuf[24] = {0};
	//sprintf(txbuf, "%d\n", moco.position);
	//HAL_UART_Transmit(&huart1, txbuf, strlen(txbuf), 50);

	//if (HAL_GetTick() > timer+500)
	//{
	//	if (moco.target == 0) moco.target = 200;
	//	else if (moco.target == 200) moco.target = 0;
	//	timer = HAL_GetTick();
	//}
}

float getSupplyVoltage()
{
	return voltage;
}

void getPhaseCurrents(float* _current)
{
	_current[0] = _current[0] * 0.9 + 0.1 * ((float)adc_dma_results[0] - (float)adc_curr_idle[0]) * 0.64453125f;
	_current[1] = _current[1] * 0.9 + 0.1 * ((float)adc_dma_results[1] - (float)adc_curr_idle[1]) * 0.64453125f;
	_current[2] = _current[2] * 0.9 + 0.1 * ((float)adc_dma_results[2] - (float)adc_curr_idle[2]) * 0.64453125f;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1)
{
	__HAL_UART_CLEAR_FLAG(huart1, 0xFF);
	if(rxbuf_index < 20)
	{
		rxbuf[rxbuf_index] = rxchar;
		rxbuf_index++;
	}
	else rxbuf_index = 0;
	if ((rxchar == '\r') || (rxchar == '\n'))
	{
		rxbuf[rxbuf_index] = 0;
		if (rxbuf_index > 1)
		{
			parseCommandandUpdate(rxbuf, rxbuf_index);
		}
		rxbuf_index = 0;
	}
}

void parseCommandandUpdate(uint8_t* buffer, uint8_t index)
{
	unsigned int _direction = 0;
	unsigned int _power = 0;
	signed int _target = 0;
	signed int _position = 0;
	unsigned int _red = 0;
	unsigned int _green = 0;
	unsigned int _blue = 0;

	switch (buffer[0])
	{
		case 'T':
			if (sscanf((char*)buffer, "T%u_%u", &_direction, &_power) == 2)
			{
				uint8_t temp_direction = (uint8_t)_direction;
				uint8_t temp_power = (uint8_t)_power;
				setMode(TORQUE_MODE);
				setDirection(temp_direction);
				setPower(temp_power);
			}
			break;
		case 'P':
			if (sscanf((char *)buffer, "P%d_%u", &_target, &_power) == 2)
			{
				uint32_t temp_target = (uint32_t)_target;
				uint8_t temp_power = (uint8_t)_power;
				setMode(POSITION_MODE);
				setTarget(temp_target);
				setPower(temp_power);
			}
			break;
		case 'C':
			if (sscanf((char *)buffer, "C%u_%u_%u", &_red, &_green, &_blue) == 3)
			{
				uint8_t red = (uint8_t)_red;
				uint8_t green = (uint8_t)_green;
				uint8_t blue = (uint8_t)_blue;
				setColor(red, green, blue, 31);
			}
			break;
		case 'G':
			if (buffer[1] == '?')
			{
				char txbuf[32] = {0};
				_power = (unsigned int)getPower();
				_position = (signed int)getPosition();
				sprintf(txbuf, "%u_%d\n", _power, _position);
				HAL_UART_Transmit(&huart1, (uint8_t *)txbuf, (uint16_t)strlen(txbuf), 50);
			}
			break;
		case '*':
			// Special SCPI commands
			if (buffer[1] == 'I')
			{
				if (buffer[2] == 'D')
				{
					if (buffer[3] == 'N')
					{
						if (buffer[4] == '?')
						{
							char txbuf[] = "Knobber\n";
							HAL_UART_Transmit(&huart1, (uint8_t *)txbuf, (uint16_t)strlen(txbuf), 50);
						}
					}
				}
			}
			break;
		default:
			break;
	}
}
