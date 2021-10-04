/*
 * stm32f3xx.h
 *
 *  Created on: Sep 25, 2021
 *      Author: alperen
 */

#ifndef INC_STM32F3XX_H_
#define INC_STM32F3XX_H_
#define __vo volatile

#include "stdio.h"
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
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
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
#define USART1_BASE			(APB2PERIPH_BASE + 0x3800)
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

typedef struct {
	__vo uint32_t CR; 				/* Clock control register,					addresses offset: 0x00*/
	__vo uint32_t CFGR; 			/* Clock configuration register,			addresses offset: 0x04*/
	__vo uint32_t CIR; 				/* Clock interrupt register,				addresses offset: 0x08*/
	__vo uint32_t APB2RSTR; 		/* APB2 peripheral reset register,			addresses offset: 0x0C*/
	__vo uint32_t APB1RSTR; 		/* APB1 peripheral reset register,			addresses offset: 0x010*/
	__vo uint32_t AHBENR; 			/* AHB peripheral clock enable register,	addresses offset: 0x14*/
	__vo uint32_t APB2ENR; 			/* APB2 peripheral clock enable register,	addresses offset: 0x18*/
	__vo uint32_t APB1ENR; 			/* APB1 peripheral clock enable register,	addresses offset: 0x1C*/
	__vo uint32_t BDCR; 			/* RTC domain control register,				addresses offset: 0x20*/
	__vo uint32_t CSR; 				/* Control/status register,					addresses offset: 0x24*/
	__vo uint32_t AHBRSTR; 			/* AHB peripheral reset register,			addresses offset: 0x28*/
	__vo uint32_t CFGR2; 			/* Clock configuration register 2,			addresses offset: 0x2c*/
	__vo uint32_t CFGR3; 			/* Clock configuration register 3,			addresses offset: 0x30*/

} RCC_RegDef_t;

/*
 * Peripheral definition structure for EXTI
 */

typedef struct{
	__vo uint32_t IMR1;				/* Interrupt mask register,					addresses offset: 0x00 */
	__vo uint32_t EMR1;				/* Event mask register,						addresses offset: 0x04 */
	__vo uint32_t RTSR1;			/* Rising trigger selection register, 		addresses offset: 0x08 */
	__vo uint32_t FTSR1;			/* Faling trigger selection register, 		addresses offset: 0x0C */
	__vo uint32_t SWIER1;			/* Software interrupt event register, 		addresses offset: 0x10 */
	__vo uint32_t PR1;				/* Pending register,				 		addresses offset: 0x14 */
}EXTI_RegDef_t;

/*
 * peripheral register definitions structure for SYSCFG
 */
typedef struct{
	__vo uint32_t CFGR1;			/*Configuration register 1,					 addresses offset: 0x00 */
	__vo uint32_t RCR;				/*SRAM protection register,					 addresses offset: 0x04 */
	__vo uint32_t EXTICR1;			/*External interrupt configuration register1,addresses offset: 0x08 */
	__vo uint32_t EXTICR2;			/*External interrupt configuration register2,addresses offset: 0x0C */
	__vo uint32_t EXTICR3;			/*External interrupt configuration register3,addresses offset: 0x10 */
	__vo uint32_t EXTICR4;			/*External interrupt configuration register4,addresses offset: 0x14 */
	__vo uint32_t CFGR2;			/*Configuration register 2,					 addresses offset: 0x18 */
}SYSCFG_Reg_Def_t;

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
#define RCC							((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLCK_EN()	(RCC-> AHBENR |= (1 << 17))
#define GPIOB_PCLCK_EN()	(RCC-> AHBENR |= (1 << 18))
#define GPIOC_PCLCK_EN()	(RCC-> AHBENR |= (1 << 19))
#define GPIOD_PCLCK_EN()	(RCC-> AHBENR |= (1 << 20))
#define GPIOE_PCLCK_EN()	(RCC-> AHBENR |= (1 << 21))
#define GPIOF_PCLCK_EN()	(RCC-> AHBENR |= (1 << 22))
#define GPIOG_PCLCK_EN()	(RCC-> AHBENR |= (1 << 23))
#define GPIOH_PCLCK_EN()	(RCC-> AHBENR |= (1 << 16))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLCK_EN()			(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLCK_EN()			(RCC -> APB1ENR |= (1 << 22))
#define I2C3_PCLCK_EN()			(RCC -> APB1ENR |= (1 << 30))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLCK_EN()			(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLCK_EN()			(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLCK_EN()			(RCC -> APB1ENR |= (1 << 15))
#define SPI4_PCLCK_EN()			(RCC -> APB2ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLCK_EN()		(RCC -> APB2ENR |= (1 << 14))
#define USART2_PCLCK_EN()		(RCC -> APB1ENR |= (1 << 17))
#define USART3_PCLCK_EN()		(RCC -> APB1ENR |= (1 << 18))
#define UART4_PCLCK_EN()		(RCC -> APB1ENR |= (1 << 19))
#define UART5_PCLCK_EN()		(RCC -> APB1ENR |= (1 << 20))

/*
 * Clock Enable Macro for SYSCFG
 */
#define SYSCFG_PCLCK_EN()		(RCC -> APB2ENR |= (1 << 0))

/*
 * Clock Disable Macros for GPIOx
 */
#define GPIOA_PCLCK_DI()	(RCC-> AHBENR &= ~(1 << 17))
#define GPIOB_PCLCK_DI()	(RCC-> AHBENR &= ~(1 << 18))
#define GPIOC_PCLCK_DI()	(RCC-> AHBENR &= ~(1 << 19))
#define GPIOD_PCLCK_DI()	(RCC-> AHBENR &= ~(1 << 20))
#define GPIOE_PCLCK_DI()	(RCC-> AHBENR &= ~(1 << 21))
#define GPIOF_PCLCK_DI()	(RCC-> AHBENR &= ~(1 << 22))
#define GPIOG_PCLCK_DI()	(RCC-> AHBENR &= ~(1 << 23))
#define GPIOH_PCLCK_DI()	(RCC-> AHBENR &= ~(1 << 16))

/*
 * Clock Disable Macros for I2Cx
 */

#define I2C1_PCLCK_DI()			(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLCK_DI()			(RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_PCLCK_DI()			(RCC -> APB1ENR &= ~(1 << 30))


/*
 * Clock Disable Macros for SPIx
 */

#define SPI1_PCLCK_DI()			(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLCK_DI()			(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLCK_DI()			(RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_PCLCK_DI()			(RCC -> APB2ENR &= ~(1 << 15))


/*
 * Clock Disable Macros for USARTx
 */

#define USART1_PCLCK_DI()		(RCC -> APB2ENR &= ~(1 << 14))
#define USART2_PCLCK_DI()		(RCC -> APB1ENR &= ~(1 << 17))
#define USART3_PCLCK_DI()		(RCC -> APB1ENR &= ~(1 << 18))
#define UART4_PCLCK_DI()		(RCC -> APB1ENR &= ~(1 << 19))
#define UART5_PCLCK_DI()		(RCC -> APB1ENR &= ~(1 << 20))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET() 		do { (RCC-> AHBENR |= (1 << 17));   (RCC-> AHBENR &= ~(1 << 17));}while(0)
#define GPIOB_REG_RESET() 		do { (RCC-> AHBENR |= (1 << 18));   (RCC-> AHBENR &= ~(1 << 18));}while(0)
#define GPIOC_REG_RESET() 		do { (RCC-> AHBENR |= (1 << 19));   (RCC-> AHBENR &= ~(1 << 19));}while(0)
#define GPIOD_REG_RESET() 		do { (RCC-> AHBENR |= (1 << 20));   (RCC-> AHBENR &= ~(1 << 20));}while(0)
#define GPIOE_REG_RESET() 		do { (RCC-> AHBENR |= (1 << 21));   (RCC-> AHBENR &= ~(1 << 21));}while(0)
#define GPIOF_REG_RESET() 		do { (RCC-> AHBENR |= (1 << 22));   (RCC-> AHBENR &= ~(1 << 22));}while(0)
#define GPIOG_REG_RESET() 		do { (RCC-> AHBENR |= (1 << 23));   (RCC-> AHBENR &= ~(1 << 23));}while(0)
#define GPIOH_REG_RESET() 		do { (RCC-> AHBENR |= (1 << 16));   (RCC-> AHBENR &= ~(1 << 16));}while(0)






/*
 * some generic macros
 */
#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET 			RESET



#endif /* INC_STM32F3XX_H_ */
