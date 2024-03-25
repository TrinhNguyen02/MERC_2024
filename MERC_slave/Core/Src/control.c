/*
 * control.c
 *
 *  Created on: Mar 3, 2024
 *      Author: tantr
 */
#include "main.h"
#include "control.h"
#include "stm32f1xx_hal.h"

uint8_t dir_1 = 0;
uint8_t dir_2 = 0;
uint8_t dir_3 = 0;
uint8_t dir_4 = 0;

void ctrl_DirMotor(uint8_t rxCtrlBuffer){
	 dir_1 = (rxCtrlBuffer & 0xC0) >> 6;
	 dir_2 = (rxCtrlBuffer & 0x30) >> 4;
	 dir_3 = (rxCtrlBuffer & 0x0C) >> 2;
	 dir_4 = (rxCtrlBuffer & 0x03) >> 0;

	 // DIRECT MOTOR 1
	 if(dir_1 == 0x02){
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
	 }
	 else if (dir_1 == 0x01) {
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
	 }
	 else{
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
	 }

	 // DIRECT MOTOR 2
	 if(dir_2 == 0x02){
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET);
	 }
	 else if (dir_2 == 0x01) {
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET);
	 }
	 else{
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET);
	 }

	 // DIRECT MOTOR 3
	 if(dir_3 == 0x02){
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
	 }
	 else if (dir_3 == 0x01) {
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
	 }
	 else{
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
	 }

	 // DIRECT MOTOR 4
	 if(dir_4 == 0x02){
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, RESET);
	 }
	 else if (dir_4 == 0x01) {
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, SET);
	 }
	 else{
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, RESET);
	 }
}
