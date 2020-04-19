/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes and Amreeta Sengupta
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>

void gpioInit()
{
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	GPIO_PinModeSet(Button_port, Button_pin, gpioModeInputPull, true);
	GPIO_PinModeSet(Button_port, Button1, gpioModeInputPull, true);
	GPIO_ExtIntConfig(Button_port,Button_pin,Button_pin,true,true,true);
	NVIC->ISER[(((uint32_t)GPIO_EVEN_IRQn) >> 5UL)] |= (uint32_t)(1UL << (((uint32_t)GPIO_EVEN_IRQn) & 0x1FUL));
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}
void gpioEnableDisplay()
{
	GPIO_PinOutSet(lcd_port,lcd_pin);
}
void gpioSetDisplayExtcomin(bool high)
{
	GPIO_PinOutToggle(lcd_port,ext_com_in);
}

void GPIO_callback(void)
{
	LOG_INFO("GPIO Callback");
	if(GPIO_PinInGet(Button_port,Button_pin) == 1)
	{
		gecko_external_signal(EVENT_PB0_FALLING);
	}
	else
	{
		gecko_external_signal(EVENT_PB0_RISING);
	}
}

