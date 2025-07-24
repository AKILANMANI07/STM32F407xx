/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jul 21, 2025
 *      Author: akilanm
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"
/*GPIO Pin Number*/
#define GPIO_PIN_0	0
#define GPIO_PIN_1	1
#define GPIO_PIN_2	2
#define GPIO_PIN_3	3
#define GPIO_PIN_4	4
#define GPIO_PIN_5	5
#define GPIO_PIN_6	6
#define GPIO_PIN_7	7
#define GPIO_PIN_8	8
#define GPIO_PIN_9	9
#define GPIO_PIN_10	10
#define GPIO_PIN_11	11
#define GPIO_PIN_12	12
#define GPIO_PIN_13	13
#define GPIO_PIN_14	15
#define GPIO_PIN_15	15

/*GPIO pin possible modes*/
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTF 		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		5
#define GPIO_MODE_IT_RT		6
#define GPIO_MODE_IT_FRT	7

/*GPIO possible Output type*/
#define GPIO_OPTYPE_PP		0
#define GPIO_OPTYPE_OD		1

/*GPIO posible output Speed Type*/
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VHIGH	3

/*GPIO pin Pull up/down type*/
#define GPIO_NPUPP			0
#define GPIO_PULLD			1
#define GPIO_PULLU			2
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_Mode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_AltFunmode;
}GPIO_PinConfig_t;
typedef struct
{
	GPIO_RegDef_t *pGPIO;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*Peripheral clock Setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *PGPIOx,uint8_t EnOrDi);

/* Init and DeInit*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *PGPIOx);

/*Read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *PGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputport(GPIO_RegDef_t *PGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *PGPIOx, uint8_t PinNumber,uint8_t Value);
void GPIO_WritToOutputPort(GPIO_RegDef_t *PGPIOx,uint16_t PortValue);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *PGPIOx, uint8_t PinNumber);

/*Interrupt Configuration and Handling*/
void GPIO_IRQITconfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
