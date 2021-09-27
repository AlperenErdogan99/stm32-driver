/*
 * stm32f3xx.h
 *
 *  Created on: Sep 25, 2021
 *      Author: alperen
 */

#ifndef INC_STM32F3XX_H_
#define INC_STM32F3XX_H_

#define __vo volatile
/*
 * base address of flash
 */
#define FLASH_BASEADDR 			0x08000000U

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASE 			0x40000000
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000
#define AHB1PERIPH_BASE			0x40020000
#define AHB2PERIPH_BASE			0x48000000
#define AHB3PERIPH_BASE			0x50000000
#define AHB4PERIPH_BASE			0x60000000

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all peripherals
 * add offset value for all peripherals that bus
 */

#define GPIOA_BASEADDR			(AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR			(AHB2PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR			(AHB2PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR			(AHB2PERIPH_BASE + 0x1800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all peripherals
 * add offset value for all peripherals that bus
 */

#define I2C1_BASE			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE 			(APB1PERIPH_BASE + 0x7800)
#define SPI2_BASE			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE			(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASE 		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASE			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE			(APB1PERIPH_BASE + 0x5000)

/*
 * Base addresses of peripherals which are hanging on  APB2 bus
 * TODO: Complete for all peripherals
 * add offset value for all peripherals that bus
 */
#define SPI1_BASE			(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASE			(APB2PERIPH_BASE + 0x3C00)
#define EXTI_BASE			(APB2PERIPH_BASE + 0x0400)
#define SYSCFG_BASE			(APB2PERIPH_BASE + 0x0000)

/*
 * Note: Registers of a peripheral are specific to MCU
 * e.g: Number of Registers of SPI peripheral of stm32f4x family mcus may be different
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 * Note: Use volatile keyword for register definition
 */

typedef struct{
	__vo uint32_t MODER;			/*	GPIO port mode register, 				addresses offset: 0x00 */
	__vo uint32_t OTYPER;			/*	GPIO port output type register, 		addresses offset: 0x04 */
	__vo uint32_t OSPEEDR;			/*	GPIO port output speed register, 		addresses offset: 0x08 */
	__vo uint32_t PUPDR;			/*	GPIO port pull-up/pull-down register, 	addresses offset: 0x0C */
	__vo uint32_t IDR;				/*	GPIO port input data register, 			addresses offset: 0x10 */
	__vo uint32_t ODR;				/*	GPIO port output data register, 		addresses offset: 0x14 */
	__vo uint32_t BSSR;				/*	GPIO port bit set/reset register, 		addresses offset: 0x18 */
	__vo uint32_t LCKR;				/*	GPIO port configuration lock register, 	addresses offset: 0x1C */
	__vo uint32_t AFR[2];			/*	GPIO alternate function registers, 		addresses offset: 0x20, 0x24 */
}GPIO_RegDef_t;

/*
 * peripheral definitons (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA 						((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 						((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 						((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 						((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 						((GPIO_RegDef_t*) GPIOH_BASEADDR)



#endif /* INC_STM32F3XX_H_ */
