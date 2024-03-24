/*
 * control.c
 *
 *  Created on: Mar 22, 2024
 *      Author: tantr
 */
#include "main.h"
#include "control.h"
#include "stm32f1xx_hal.h"


uint8_t dirBuffer[2];

extern UART_HandleTypeDef huart1;

extern int PWM;
extern double error;
extern int output;

extern motorStruct ctrlMotor_1;
extern motorStruct ctrlMotor_2;
extern motorStruct ctrlMotor_3;
extern motorStruct ctrlMotor_4;


void constrain(int* value, int min, int max) {
    if (*value <= min) {
        *value = min;
    } else if (*value >= max) {
        *value = max;
    }
}
int calcPD(int input, int setPoint, double kp, double kd) {
    static int previous_error = 0;
    static int previous_setPoint = 0;
    error = (setPoint - input)*MAX_PWM/MAX_SPEED;
    if((setPoint - previous_setPoint) >= 100) {
    	setPoint-= 50;
    }
    else if((setPoint - previous_setPoint) <= -100) {
    	setPoint+= 50;
    }
    previous_setPoint = setPoint;


    output = setPoint*MAX_PWM/MAX_SPEED + kp * error + kd * (error - previous_error);

    previous_error = error;
    constrain(&output, -100, 100);
    return output;
}
void ctrlMotor(motorStruct Motor_x, int16_t throttle, int16_t speed, double kp, double kd){
	PWM = calcPD(speed, throttle, kp, kd);
	// channel B of tb6612 --> tim2 channel 2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, SET);
	if( PWM == 0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, RESET);

	if (PWM > 0 ){
		// dir = 1 0 ~ 0x02
		Motor_x.dirMotor = 0x02;
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, RESET);
	    __HAL_TIM_SetCompare(Motor_x.htim,Motor_x.Channel,PWM*10);
	}
	else if (PWM < 0 ){
		// dir = 0 1 ~ 0x01
		Motor_x.dirMotor = 0x01;

//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);
	    __HAL_TIM_SetCompare(Motor_x.htim,Motor_x.Channel,PWM*(-10));
	}
}

void ctrlDir(void){
	dirBuffer[0] = 0xFF;
	dirBuffer[1] |= (ctrlMotor_1.dirMotor << 6) |  (ctrlMotor_2.dirMotor << 4) | (ctrlMotor_3.dirMotor << 2) | (ctrlMotor_4.dirMotor << 0);
	HAL_UART_Transmit(&huart1, dirBuffer, 2, 10);
}
