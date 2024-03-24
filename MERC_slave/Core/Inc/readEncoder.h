/*
 * readEncoder.h
 *
 *  Created on: Mar 19, 2024
 *      Author: tantr
 */

#ifndef INC_READENCODER_H_
#define INC_READENCODER_H_

uint8_t readEnc(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_A, uint16_t GPIO_Pin_B);
void getCounter(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_A, uint16_t GPIO_Pin_B, int16_t *counter);


#endif /* INC_READENCODER_H_ */
