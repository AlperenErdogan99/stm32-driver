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


/*******************************************************************
 * API supported by this driver
 * For more information about the APIs check the function definitions
 *******************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToogleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F3XX_GPIO_DRIVER_H_ */
