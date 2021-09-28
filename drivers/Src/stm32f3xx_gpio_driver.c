/*
 * stm32f3xx_gpio_driver.c
 *
 *  Created on: Sep 27, 2021
 *      Author: alperen
 */
#include "stm32f3xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*******************************
 *
 * @fn			- GPIO_PeripheralClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 * @Note		- none
 *
 */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLCK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLCK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLCK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLCK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLCK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLCK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLCK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLCK_EN();
		}

	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLCK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLCK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLCK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLCK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLCK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLCK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLCK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLCK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/*******************************
 *
 * @fn			- GPIO_PeripheralClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 * @Note		- none
 * TODO: 1. configure the mode of pin
 * 		 2. configure the speed
 * 		 3. configure the pupd settings
 * 		 4. configure the optype
 * 		 5. configure the alt functionality
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){


}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

}

void GPIO_ToogleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){


}

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){


}

void GPIO_IRQHandling(uint8_t PinNumber){


}


