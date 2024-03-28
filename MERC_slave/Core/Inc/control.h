/*
 * control.h
 *
 *  Created on: Mar 3, 2024
 *      Author: tantr
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_
#include "main.h"
#include "stm32f1xx_hal.h"

void ctrl_DirMotor(uint8_t dirMotor);
int16_t read_Speed(TIM_HandleTypeDef *htim);

#endif /* INC_CONTROL_H_ */
