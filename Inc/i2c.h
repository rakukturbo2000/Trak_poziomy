/*
 * i2c.h
 *
 *  Created on: 18 mar 2023
 *      Author: Ja
 */
#ifndef INC_I2C_H_
#define INC_I2C_H_
#include "stm32f1xx.h"

void InitI2C();
void TransmitI2C (uint8_t adres,uint8_t* data,uint8_t lenght);



#endif /* INC_I2C_H_ */
