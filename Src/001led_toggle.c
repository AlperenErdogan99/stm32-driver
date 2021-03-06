/*
 * 001led_toggle.c
 *
 *  Created on: Sep 30, 2021
 *      Author: alperen
 */

#include "stm32f3xx.h"
#include "stm32f3xx_gpio_driver.h"
#include "stdio.h"
void delay(void){
	for (uint32_t i=0 ; i<500000 ; i++);
}
int main(void){

	GPIO_Handle_t GpioLed ;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeripheralClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	while(1){
		GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}


	return 0;
}
