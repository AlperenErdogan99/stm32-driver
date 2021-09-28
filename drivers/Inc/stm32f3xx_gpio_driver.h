/*
 * stm32f3xx_gpio_driver.h
 *
 *  Created on: Sep 27, 2021
 *      Author: alperen
 */

#ifndef INC_STM32F3XX_GPIO_DRIVER_H_
#define INC_STM32F3XX_GPIO_DRIVER_H_

#include "stm32f3xx.h"
/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct{

	GPIO_RegDef_t *pGPIOx; /* This holds the  base address of the GPIO port which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig; /* This holds GPIO pin configuration settings */

}GPIO_Handle_t;
#endif /* INC_STM32F3XX_GPIO_DRIVER_H_ */
