/*
 * control.c
 *
 *  Created on: Mar 22, 2024
 *      Author: tantr
 */
#include "main.h"
#include "control.h"
#include "stm32f1xx_hal.h"


extern UART_HandleTypeDef huart2;


extern motorStruct Motor_1;
extern motorStruct Motor_2;
extern motorStruct Motor_3;
extern motorStruct Motor_4;

extern TIM_HandleTypeDef htim1;

// ---------------------------

uint8_t dirBuffer[2] = {0, 0};

void constrain(int16_t* value, int min, int max) {
    if (*value <= min) {
        *value = min;
    } else if (*value >= max) {
        *value = max;
    }
}
int16_t calcPD(motorStruct* Motor_x, int16_t input, int16_t setPoint) {
    int16_t error = (int16_t)((setPoint - input)*MAX_PWM/MAX_SPEED);
    Motor_x->error = error;
    int8_t kp = 1;
    int8_t kd = 1;
    if((setPoint - Motor_x->preSetPoint) >= 100) {
    	setPoint-= 50;
    }
    else if((setPoint - Motor_x->preSetPoint) <= -100) {
    	setPoint+= 50;
    }
    Motor_x->preSetPoint = setPoint;


    int16_t output = (int16_t)(setPoint*MAX_PWM/MAX_SPEED + kp * error + kd * (error - Motor_x->preError));

    Motor_x->preSetPoint = setPoint;
    Motor_x->preError = (int16_t)error;
    constrain(&output, -100, 100);
    return output;
}
void ctrlMotor(motorStruct* Motor_x){
	Motor_x->PWM = calcPD(Motor_x, *(Motor_x->Speed), Motor_x->setPointSpeed);
	int16_t PWM = Motor_x->PWM;
	if( PWM == 0) {
		Motor_x->dirMotor = 0x00; // dir = 0 1 ~ 0x01
	    __HAL_TIM_SetCompare(Motor_x->htim, (uint32_t)(Motor_x->Channel), 0);
	}

	if (PWM > 0 ){
		Motor_x->dirMotor = 0x02; // dir = 1 0 ~ 0x02
	    __HAL_TIM_SetCompare(Motor_x->htim, (uint32_t)(Motor_x->Channel), PWM*10);
	}
	else if (PWM < 0 ){
		Motor_x->dirMotor = 0x01; // dir = 0 1 ~ 0x01
	    __HAL_TIM_SetCompare(Motor_x->htim, (uint32_t)(Motor_x->Channel),PWM*(-10));
	}
}
void ctrlDir(void){
	dirBuffer[0] = 0xFF;
	dirBuffer[1] = 0x00;
	dirBuffer[1] |= (Motor_1.dirMotor << 6) | (Motor_2.dirMotor << 4) | (Motor_3.dirMotor << 2) | (Motor_4.dirMotor << 0);
	HAL_UART_Transmit(&huart2, dirBuffer, 2, 10);

}

//---------------------------
