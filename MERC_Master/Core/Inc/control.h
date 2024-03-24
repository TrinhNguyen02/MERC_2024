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

void constrain(int* value, int min, int max);
int calcPD(int input, int setPoint, double kp, double kd);
void ctrlMotor(motorStruct Motor_x, int16_t throttle, int16_t speed, double kp, double kd);
#endif /* INC_CONTROL_H_ */
