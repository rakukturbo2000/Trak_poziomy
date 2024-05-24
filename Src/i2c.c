/*
 * i2c.c
 *
 *  Created on: 18 mar 2023
 *      Author: Ja
 */
#include "i2c.h"
#include "stm32f1xx.h"

   void InitI2C() {

	  I2C1->CR1|=I2C_CR1_PE;
	  I2C1->CR2|=I2C_CR2_FREQ_2|I2C_CR2_FREQ_5;
	  I2C1->CCR|=I2C_CCR_CCR_Msk&0xb4;
	  I2C1->TRISE=I2C_TRISE_TRISE_Msk&0x25;

  }
void TransmitI2C (uint8_t adres,uint8_t* data,uint8_t lenght){
	uint32_t temp;
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));


	(void)I2C1->SR1;
	I2C1->DR=adres&0b11111110;
	while (!(I2C1->SR1 & I2C_SR1_ADDR));


	temp=I2C1->SR2;
	while(!(I2C1->SR1&I2C_SR1_TXE));
	for(uint8_t i=0;i<lenght;i++){
		I2C1->DR=*data++;
		while(!(I2C1->SR1 & I2C_SR1_TXE));
	}
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	I2C1->CR1|=I2C_CR1_STOP;

}


