/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 21, 2025
 *      Author: akilanm
 */


#include<stdint.h>
#include "stm32f407xx_gpio_driver.h"


/*Peripheral clock Setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *PGPIOx,uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{

			if(PGPIOx== GPIOA)
			{
						GPIOA_PCLK_EN;
			}
			else if(PGPIOx== GPIOB)
			{
						GPIOB_PCLK_EN;
			}
			else if(PGPIOx== GPIOC)
			{
						GPIOC_PCLK_EN;
			}
			else if(PGPIOx== GPIOD){
						GPIOD_PCLK_EN;
			}
			else if(PGPIOx== GPIOE)
			{
						GPIOE_PCLK_EN;
			}
			else if(PGPIOx== GPIOF)
			{
						GPIOF_PCLK_EN;
			}
			else if(PGPIOx== GPIOG)
			{
						GPIOG_PCLK_EN;
			}
			else if(PGPIOx== GPIOH)
			{
						GPIOH_PCLK_EN;
			}
			else
			{
						GPIOI_PCLK_EN;
			}

	}
	else
	{
					if(PGPIOx== GPIOA)
					{
								GPIOA_PCLK_DI;
					}
					else if(PGPIOx== GPIOB)
					{
								GPIOB_PCLK_DI;
					}
					else if(PGPIOx== GPIOC)
					{
								GPIOC_PCLK_DI;
					}
					else if(PGPIOx== GPIOD){
								GPIOD_PCLK_DI;
					}
					else if(PGPIOx== GPIOE)
					{
								GPIOE_PCLK_DI;
					}
					else if(PGPIOx== GPIOF)
					{
								GPIOF_PCLK_DI;
					}
					else if(PGPIOx== GPIOG)
					{
								GPIOG_PCLK_DI;
					}
					else if(PGPIOx== GPIOH)
					{
								GPIOH_PCLK_DI;
					}
					else
					{
								GPIOI_PCLK_DI;
					}
	}
}

/* Init and DeInit*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//1. configure mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode<=GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		temp= pGPIOHandle->GPIO_PinConfig.GPIO_Mode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIO->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing corresponding 2 bits
		pGPIOHandle->pGPIO->MODER |= temp; //setting
	}
	else
	{
		//TODO: for Interrupt mode
	}

	//2. configure speed
	temp=0;
	temp= pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OSPEEDR |= temp;

	//3. configure pull up pull down
	temp=0;
	temp= pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->PUPDR |= temp;

	//4. configure the output type
	temp=0;
	temp= pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OTYPER |= temp;

	// 5. configure alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode<=GPIO_MODE_ALTF)
	{
		temp=0;
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<GPIO_PIN_8)
		{
			temp= pGPIOHandle->GPIO_PinConfig.GPIO_AltFunmode << 4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIO->AFRL &= ~(0x0F << 4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIO->AFRL |= temp;
		}
		else
		{
			temp= pGPIOHandle->GPIO_PinConfig.GPIO_AltFunmode << 4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8);
			pGPIOHandle->pGPIO->AFRH &= ~(0x0F << 4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8));
			pGPIOHandle->pGPIO->AFRH |= temp;
		}
	}
}
void GPIO_DeInit(GPIO_RegDef_t *PGPIOx)
{
	/*resetting GPIO pheripherals*/
	if(PGPIOx== GPIOA)
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x0);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x0);

	}
	else if(PGPIOx== GPIOB)
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x1);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x1);
	}
	else if(PGPIOx== GPIOC)
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x2);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x2);
	}
	else if(PGPIOx== GPIOD)
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x3);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x3);
	}
	else if(PGPIOx== GPIOE)
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x4);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x4);
	}
	else if(PGPIOx== GPIOF)
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x5);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x5);
	}
	else if(PGPIOx== GPIOG)
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x6);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x6);
	}
	else if(PGPIOx== GPIOH)
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x7);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x7);
	}
	else
	{
		(RCC)->RCC_APB1RSTR |= (0x1<<0x8);
		(RCC)->RCC_APB1RSTR &= ~(0x1<<0x8);
	}
}

/*Read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *PGPIOx, uint8_t PinNumber)
{
	uint8_t value=0;
	value=(uint8_t)(PGPIOx->IDR)>>PinNumber & 0x1;
	return value;
}
uint16_t GPIO_ReadFromInputport(GPIO_RegDef_t *PGPIOx)
{
		uint16_t value=0;
		value=(uint16_t)(PGPIOx->IDR);
		return value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *PGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if((PGPIOx->MODER >> 2*PinNumber)==0x01) //only output mode we need to write
	{
		PGPIOx->ODR &= ~(1<<PinNumber);
		PGPIOx->ODR = (uint32_t)(Value<<PinNumber);
	}

}
void GPIO_WritToOutputPort(GPIO_RegDef_t *PGPIOx,uint16_t PortValue)
{
	PGPIOx->ODR = (uint32_t)PortValue;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *PGPIOx, uint8_t PinNumber)
{
	PGPIOx->ODR ^= (1<<PinNumber);
}

/*Interrupt Configuration and Handling*/
void GPIO_IRQconfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
