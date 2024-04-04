/*
 * application.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Lukas
 */

#include "application.h"

#define AS5600L_ADDR 0x80

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;

uint8_t power = 0;
uint16_t angle = 0;
uint16_t new_angle = 0;
float voltage = 0;
volatile bool enable = true;
volatile bool faultstate = false;
uint8_t received[2] = {0};

uint16_t adc_dma_results[5];

volatile uint16_t phaseangle = 0;
uint8_t Buffer[25] = {0};



void init()
{
	// set PWM values to 0
	HAL_GPIO_WritePin(Enable_U_GPIO_Port, Enable_U_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Enable_V_GPIO_Port, Enable_V_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Enable_W_GPIO_Port, Enable_W_Pin, GPIO_PIN_RESET);

	// wakeup driver chip
	HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);

	// setup AS5600L to output analog value
	//HAL_I2C_Mem_Write(&hi2c2, AS5600L_ADDR, 0x08, 1, &0b00000000, 1, 1000)


	setColor(0, 0, 128, 32);
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	setAngle(0);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_results, 5);
}


void mainloop()
{
	/*
	for(uint16_t i = 0; i < 3600; i++)
	{
		setAngle(i);
		//setPower(i);
		//HAL_Delay(1);
		voltage = getSupplyVoltage();
		uint8_t test[] = "test";
		HAL_UART_Transmit(&huart1, test, 5, 1000);
	}
	*/
	//HAL_I2C_Mem_Read(&hi2c2, AS5600L_ADDR, 0x0C, I2C_MEMADD_SIZE_8BIT, received, 2, 1000);
	//new_angle = received[1] + ((uint16_t)received[0] << 8);
	//angle = (angle*7) % 4096;

	//setAngle((phaseangle+angle) % 4096);

	//setAngle(angle);

	//HAL_UART_Transmit(&huart1, received, 1, 1000);
	HAL_UART_Transmit(&huart1, (uint8_t *)"START\n", 6, 1000);

	power = 128;
	HAL_Delay(500);
	for (uint16_t x = 0; x < 3600; x+=10)
	{
		setAngle((x*7) % 3600);
		HAL_Delay(100);
		HAL_I2C_Mem_Read(&hi2c2, AS5600L_ADDR, 0x0C, I2C_MEMADD_SIZE_8BIT, received, 2, 1000);
		new_angle = received[1] + ((uint16_t)received[0] << 8);
		sprintf(Buffer, "%u;%u\n", x, new_angle);
		HAL_UART_Transmit(&huart1, Buffer, strlen(Buffer), 10000);
		//HAL_UART_Transmit(&huart1, (uint8_t *)'\n', 1, 1000);
	}
	power = 0;
	enable = false;


	while (true);
}

float getSupplyVoltage()
{
	voltage = adc_dma_results[3] / 125.27;
	return voltage;
}


void setAngle(uint16_t anglesetting)
// angle is tenths of a degree
{
	/*
	if(anglesetting > sintablen)
	{
		anglesetting = sintablen;
	}
	*/
	anglesetting = 3600 - anglesetting;

	htim1.Instance->CCR1 = ((sintab[(anglesetting)      % sintablen] * power) >> 7) + 1023;
	htim1.Instance->CCR2 = ((sintab[(anglesetting+1200) % sintablen] * power) >> 7) + 1023;
	htim1.Instance->CCR3 = ((sintab[(anglesetting+2400) % sintablen] * power) >> 7) + 1023;
}

void setPower(uint8_t powersetting)
{
	if(powersetting > 127)
	{
		powersetting = 127;
	}
	power = powersetting;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (enable)
	{
		// enable all phases
		HAL_GPIO_WritePin(Enable_U_GPIO_Port, Enable_U_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Enable_V_GPIO_Port, Enable_V_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Enable_W_GPIO_Port, Enable_W_Pin, GPIO_PIN_SET);
	}
	else
	{
		// disable all phases
		HAL_GPIO_WritePin(Enable_U_GPIO_Port, Enable_U_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Enable_V_GPIO_Port, Enable_V_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Enable_W_GPIO_Port, Enable_W_Pin, GPIO_PIN_RESET);
	}

	faultstate = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);



	//setAngle((uint16_t)adc_dma_results[4] * 0.87890625);
}
