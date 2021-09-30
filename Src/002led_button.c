/*
 * 002led_button.c
 *
 *  Created on: Sep 30, 2021
 *      Author: alperen
 */

#include "stm32f3xx.h"
#include "stm32f3xx_gpio_driver.h"
#include "stdio.h"

#define LOW DISABLE
#define BTN_PRESSED LOW //when use discovery board use high. Because pull up down resistor

void delay(void){
	for (uint32_t i=0 ; i<250000 ; i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn ;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeripheralClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED){
			delay(); //for debounce
			GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}


	return 0;
}
