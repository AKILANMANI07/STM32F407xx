/*
 * stm32f407xx.h
 *
 *  Created on: Jul 21, 2025
 *      Author: akilanm
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __vol volatile


#define FLASH_BASEADDR				0X08000000U
#define SRAM1_BASEADDR 				0X20000000U
#define SRAM2_BASEADDR				0X2001C000U

/*AHB and APB peripheral base address*/

#define AHB1BUS_BASEADDR			(void *)0x40020000U
#define AHB2BUS_BASEADDR			0X50000000U
#define AHB3BUS_BASEADDR			0XA0000000U
#define APB1BUS_BASEADDR			0X40000000U
#define APB2BUS_BASEADDR			0X40010000U

/* Base address of peripheral which are hanging on AHB1 bus*/
#define GPIOA_BASEADDR 				AHB1BUS_BASEADDR+0x0000
#define GPIOB_BASEADDR 				AHB1BUS_BASEADDR+0x0400
#define GPIOC_BASEADDR 				AHB1BUS_BASEADDR+0x0800
#define GPIOD_BASEADDR 				AHB1BUS_BASEADDR+0x0C00
#define GPIOE_BASEADDR 				AHB1BUS_BASEADDR+0x1000
#define GPIOF_BASEADDR 				AHB1BUS_BASEADDR+0x1400
#define GPIOG_BASEADDR 				AHB1BUS_BASEADDR+0x1800
#define GPIOH_BASEADDR 				AHB1BUS_BASEADDR+0x1C00
#define GPIOI_BASEADDR 				AHB1BUS_BASEADDR+0x2000
#define GPIOJ_BASEADDR 				AHB1BUS_BASEADDR+0x2400
#define GPIOK_BASEADDR 				AHB1BUS_BASEADDR+0x2800
#define RCC_BASEADDR 				AHB1BUS_BASEADDR+0x3800

/* Base address of peripheral which are hanging on APB1 bus*/
#define I2C1_BASEADDR 				APB1BUS_BASEADDR+0x5400
#define I2C2_BASEADDR 				APB1BUS_BASEADDR+0x5800
#define I2C3_BASEADDR 				APB1BUS_BASEADDR+0x5C00
#define SPI2_BASEADDR 				APB1BUS_BASEADDR+0x3800
#define SPI3_BASEADDR 				APB1BUS_BASEADDR+0x3C00
#define USART2_BASEADDR 			APB1BUS_BASEADDR+0x4400
#define USART3_BASEADDR 			APB1BUS_BASEADDR+0x4800
#define UART4_BASEADDR 				APB1BUS_BASEADDR+0x4C00
#define UART5_BASEADDR 				APB1BUS_BASEADDR+0x5000

/* Base address of peripheral which are hanging on APB1 bus*/

#define SPI1_BASEADDR				APB2BUS_BASEADDR+0x3000
#define SPI4_BASEADDR				APB2BUS_BASEADDR+0x3400
#define USART1_BASEADDR				APB2BUS_BASEADDR+0x1000
#define USART6_BASEADDR				APB2BUS_BASEADDR+0x1400
#define EXTI_BASEADDR				APB2BUS_BASEADDR+0x3C00
#define SYSCFG_BASEADDR				APB2BUS_BASEADDR+0x3800

/*GPIO structure*/
typedef struct
{
	__vol uint32_t MODER; 		/*GPIOx_MODER*/
	__vol uint32_t OTYPER;		/*GPIOx_OTYPER*/
	__vol uint32_t OSPEEDR;		/*GPIOx_OSPEEDR*/
	__vol uint32_t PUPDR;		/*GPIOx_PUPDR*/
	__vol uint32_t IDR;			/*GPIOx_IDR*/
	__vol uint32_t ODR;			/*GPIOx_ODR*/
	__vol uint32_t BSRR;		/*GPIOx_BSRR*/
	__vol uint32_t LCKR;		/*GPIOx_LCKR*/
	__vol uint32_t AFRL;		/*GPIOx_AFRL*/
	__vol uint32_t AFRH;		/*GPIOx_AFRH*/
}GPIO_RegDef_t;

typedef struct
{
	uint32_t RCC_CR;
	uint32_t RCC_PLLCFGR;
	uint32_t RCC_CFGR;
	uint32_t RCC_CIR;
	uint32_t RCC_AHB1RSTR;
	uint32_t RCC_AHB2RSTR;
	uint32_t RCC_AHB3RSTR;
	uint32_t RESERVED;			/*reserved 0x1C*/
	uint32_t RCC_APB1RSTR;
	uint32_t RCC_APB2RSTR;
	uint32_t RESERVED1[2];		/*reserved 0x28-0x2C*/
	uint32_t RCC_AHB1ENR;
	uint32_t RCC_AHB2ENR;
	uint32_t RCC_AHB3ENR;
	uint32_t RESERVER2;			/*reserved 0x3C*/
	uint32_t RCC_APB1ENR;
	uint32_t RCC_APB2ENR;
	uint32_t RESERVED3[2];		/*reserved 0x48-0x4C*/
	uint32_t RCC_AHB1LPENR;
	uint32_t RCC_AHB2LPENR;
	uint32_t RCC_AHB3LPENR;
	uint32_t RESERVED4;			/*reserved 0x5C*/
	uint32_t RCC_APB1LPENR;
	uint32_t RCC_APB2LPENR;
	uint32_t RESERVED5[2];		/*reserved 0x68-0x6C*/
	uint32_t RCC_BDCR;
	uint32_t RCC_CSR;
	uint32_t RESERVED6[2];		/*reserved 0x78-0x7C*/
	uint32_t RCC_SSCGR;
	uint32_t RCC_PLLI2SCFGR;

}RCC_RegDef_t;

#define RCC			(RCC_RegDef_t*)RCC_BASEADDR
#define GPIOA 		(GPIO_RegDef_t*)GPIOA_BASEADDR
#define GPIOB 		(GPIO_RegDef_t*)GPIOB_BASEADDR
#define GPIOC 		(GPIO_RegDef_t*)GPIOC_BASEADDR
#define GPIOD 		(GPIO_RegDef_t*)GPIOD_BASEADDR
#define GPIOE 		(GPIO_RegDef_t*)GPIOE_BASEADDR
#define GPIOF 		(GPIO_RegDef_t*)GPIOF_BASEADDR
#define GPIOG 		(GPIO_RegDef_t*)GPIOG_BASEADDR
#define GPIOH 		(GPIO_RegDef_t*)GPIOH_BASEADDR
#define GPIOI 		(GPIO_RegDef_t*)GPIOI_BASEADDR
/*clock enable macro for GPIOx peripherals */
#define GPIOA_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<0)
#define GPIOB_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<1)
#define GPIOC_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<2)
#define GPIOD_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<3)
#define GPIOE_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<4)
#define GPIOF_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<5)
#define GPIOG_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<6)
#define GPIOH_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<7)
#define GPIOI_PCLK_EN (RCC)->RCC_AHB1RSTR|=(1<<8)

/*clock enable macro for GPIOx peripherals */
#define GPIOA_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<0)
#define GPIOB_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<1)
#define GPIOC_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<2)
#define GPIOD_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<3)
#define GPIOE_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<4)
#define GPIOF_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<5)
#define GPIOG_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<6)
#define GPIOH_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<7)
#define GPIOI_PCLK_DI (RCC)->RCC_AHB1RSTR &=~(1<<8)

#define ENABLE 	1
#define DISABLE 0
#define SET		1
#define RESET	0
#endif /* INC_STM32F407XX_H_ */
