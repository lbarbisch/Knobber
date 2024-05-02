/*
 * AP102_Driver.c
 *
 *  Created on: Nov 19, 2023
 *      Author: Lukas
 */

#include "AP102_Driver.h"

void setColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
// max brightness is 31
{
  for (int i = 0; i<32; i++)
  {
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_RESET);
  }
  brightness |= 0xE0;
  for (int i = 0; i<8; i++)
  {
	  if (brightness & (0b10000000 >> i))
	  {
		  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_RESET);
	  }
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_RESET);
  }
  for (int i = 0; i<8; i++)
  {
	  if (blue & (0b10000000 >> i))
	  {
		  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_RESET);
	  }
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_RESET);
  }
  for (int i = 0; i<8; i++)
  {
	  if (green & (0b10000000 >> i))
	  {
		  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_RESET);
	  }
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_RESET);
  }
  for (int i = 0; i<8; i++)
  {
	  if (red & (0b10000000 >> i))
	  {
		  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_RESET);
	  }
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_RESET);
  }

  HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_RESET);
}



void sweepBrightness(uint8_t red, uint8_t green, uint8_t blue)
{
	for (uint8_t i = 0; i < 32; i++)
	{
		setColor(red, green, blue, i);
		HAL_Delay(10);
	}
	for (int i = 31; i > 0; i--)
	{
		setColor(red, green, blue, i);
		HAL_Delay(10);
	}
}
