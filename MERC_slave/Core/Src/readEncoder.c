/*
 * readEncoder.c
 *
 *  Created on: Mar 19, 2024
 *      Author: tantr
 */
#include "main.h"


//uint8_t readEnc(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_A, uint16_t GPIO_Pin_B){
//	static uint8_t preEncA = 0;
//	static uint8_t preEncB = 0;
//	uint8_t encA = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin_A);
//	uint8_t encB = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin_B);
//
//	uint8_t decoder = 0;
//	decoder |= (preEncA << 3) |  (encA << 2) | (preEncB << 1) | (encB << 0) ;
//
//	preEncA = encA;
//	preEncB = encB;
//	return decoder;
//}
//void getCounter(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_A, uint16_t GPIO_Pin_B, uint16_t *counter){
//	switch(readEnc(GPIOx, GPIO_Pin_A, GPIO_Pin_B)){
//	case 0x04:
//		*counter +=1;
//		break;
//
//	case 0x07:
//		*counter -=1;
//		break;
//
//	case 0x0B:
//		*counter +=1;
//		break;
//
//	case 0x08:
//		*counter -=1;
//		break;
//
//
//	case 0x0D:
//		*counter +=1;
//		break;
//
//	case 0x01:
//		*counter -=1;
//		break;
//
//	case 0x02:
//		*counter +=1;
//		break;
//
//	case 0x0E:
//		*counter -=1;
//		break;
//	default:
//		break;
//	}
//}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1 ){
//		uint16_t tickCnt = 500;
//		while(tickCnt != 0){
//			tickCnt--;
//		}
//		getCounter(GPIOA, GPIO_PIN_0, GPIO_PIN_1, &counter_1);
//	}
//	else if(GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_3 ){
//		uint16_t tickCnt = 500;
//		while(tickCnt != 0){
//			tickCnt--;
//		}
//		getCounter(GPIOA, GPIO_PIN_2, GPIO_PIN_3, &counter_2);
//	}
//	else if(GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_5 ){
//		uint16_t tickCnt = 500;
//		while(tickCnt != 0){
//			tickCnt--;
//		}
//		getCounter(GPIOA, GPIO_PIN_4, GPIO_PIN_5, &counter_3);
//	}
//	else if(GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_7 ){
//		uint16_t tickCnt = 500;
//		while(tickCnt != 0){
//			tickCnt--;
//		}
//		getCounter(GPIOA, GPIO_PIN_6, GPIO_PIN_7, &counter_4);
//	}
//}

