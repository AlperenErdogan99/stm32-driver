/*
 * stm32f3xx.h
 *
 *  Created on: Sep 25, 2021
 *      Author: alperen
 */

#ifndef INC_STM32F3XX_H_
#define INC_STM32F3XX_H_

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







#endif /* INC_STM32F3XX_H_ */
