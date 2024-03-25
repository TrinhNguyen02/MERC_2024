/*
 * control.h
 *
 *  Created on: Mar 22, 2024
 *      Author: tantr
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_
#include "main.h"
#include "stm32f1xx_hal.h"

void constrain(int16_t* value, int min, int max);
int16_t calcPD(motorStruct* Motor_x, int16_t input, int16_t setPoint);
void ctrlMotor(motorStruct* Motor_x);
void ctrlDir(void);


#endif /* INC_CONTROL_H_ */
