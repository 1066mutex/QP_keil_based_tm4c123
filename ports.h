/*
 * @File: port.h
 * @Author: eddie.mhako 
 * @Date: 2021-06-24 20:26:35 
 * @Last Modified by: eddie.mhako
 * @Last Modified time: 2021-06-24 21:53:14
 * 
 * These are the port definitions for the tm4c123 and 
 * MKII BOOSER PACK attached. 
 */
#ifndef PORTS_H
#define PORTS_H
#include "bsp.h"

/* LEDs and Switches of the EK-TM4C123GXL board ..........................*/
/* pins */
#define LED_RED   (1U << 1)
#define LED_GREEN (1U << 3)
#define LED_BLUE  (1U << 2)

#define BTN_SW1   (1U << 4)
#define BTN_SW2   (1U << 0)
/* ports */
#define BTN_SWS_PORT   (GPIOF)
#define LEDS_PORT      (GPIOF)

/* LEDs and Switches of the MKII BoosterPack board.........................*/
/* pins */
#define BSTR_LED_RED   (1U << 3)
#define BSTR_LED_GREEN (1U << 3)
#define BSTR_LED_BLUE  (1U << 4)

#define BSTR_BTN_SW3   (1U << 6)
#define BSTR_BTN_SW4   (1U << 7)

// ports-------------------------

#define BSTR_LED_RED_PORT   (GPIOF)  //! this is used on the tm4c123.
#define BSTR_LED_GREEN_PORT (GPIOB)
#define BSTR_LED_BLUE_PORT  (GPIOC)
#define BSTR_BTN_SWS_PORT   (GPIOD)





#endif /* PORTS_H */
