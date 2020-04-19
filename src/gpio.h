/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include "log.h"

#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1
#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1
#define	lcd_port (gpioPortD)
#define lcd_pin (15)
#define ext_com_in (13)
#define EVENT_PB0_RISING (1U << 1)
#define EVENT_PB0_FALLING (1U << 2)
#define	LED0_port gpioPortF
#define LED0_pin 4
#define LED1_port gpioPortF
#define LED1_pin 5
#define Button_port gpioPortF
#define Button_pin 6
#define Button1 7


void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void gpioEnableDisplay();
void GPIO_callback(void);
void gpioSetDisplayExtcomin(bool high);
#endif /* SRC_GPIO_H_ */
