/*
 * stm32f3xx_gpio_driver.c
 *
 *  Created on: Sep 27, 2021
 *      Author: alperen
 */
#include "stm32f3xx.h"
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0;
	//configure the mode of pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear the set area
		pGPIOHandle->pGPIOx->MODER |= temp;

	} else {
		//interrupt mode. this part will code later
	}

	// configure the speed of pin
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// configure the pull up pull down of pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// configure the output type of pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType
			<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//configure the alternate function of the pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN) {
		uint8_t temp1;
		uint8_t temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |=
				(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}

}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		// write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	 pGPIOx->ODR = Value;
}

void GPIO_ToogleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){


}

void GPIO_IRQHandling(uint8_t PinNumber){


}


