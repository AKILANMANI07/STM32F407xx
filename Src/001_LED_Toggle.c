/*
 * 001_LED_Toggle.c
 *
 *  Created on: Jul 22, 2025
 *      Author: akilanm
 */
#include<stdint.h>
#include<unistd.h>
#include"stm32f407xx_gpio_driver.h"
void delay(void)
{
	for(uint32_t i=0; i<50000;i++);
}
int main()
{
	GPIO_Handle_t GPIO_Led;
	GPIO_Led.pGPIO=GPIOD;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber=12;
	GPIO_Led.GPIO_PinConfig.GPIO_Mode=GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OPTYPE_PP;//push-pull configuration
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GPIO_Led.GPIO_PinConfig.GPIO_PuPdControl=GPIO_PULLU;
	GPIO_PeriClockControl(GPIO_Led.pGPIO,ENABLE);
	GPIO_Init(&GPIO_Led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIO_Led.pGPIO,GPIO_Led.GPIO_PinConfig.GPIO_PinNumber );
		delay();
	}

	return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
}


