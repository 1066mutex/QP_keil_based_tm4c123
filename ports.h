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

/* GPIO port numbers definition for run mode clock gating...*/
#define PORTA               (1U << 0)
#define PORTB               (1U << 1)
#define PORTC               (1U << 2)
#define PORTD               (1U << 3)
#define PORTE               (1U << 4)
#define PORTF               (1U << 5)

/* LEDs and Switches of the EK-TM4C123GXL board ..........................*/
/* pins */
#define LED_RED            (1U << 1)
#define LED_GREEN          (1U << 3)
#define LED_BLUE           (1U << 2)
         
#define BTN_SW1            (1U << 4)
#define BTN_SW2            (1U << 0)
/* ports */
#define BTN_SWS_PORT       (GPIOF)
#define LEDS_PORT          (GPIOF)

/* LEDs and Switches of the MKII BoosterPack board.........................*/
/* pins */
#define BSTR_LED_RED       (1U << 3) // PF3
#define BSTR_LED_GREEN     (1U << 3) // PB3
#define BSTR_LED_BLUE      (1U << 4) // PC4
 //    
#define BSTR_BTN_SW3       (1U << 6) // PD6
#define BSTR_BTN_SW4       (1U << 7) // PD7

// ports-------------------------

#define BSTR_LED_RED_PORT   (GPIOF) // PF //! this is used on the tm4c123.
#define BSTR_LED_GREEN_PORT (GPIOB) // PB
#define BSTR_LED_BLUE_PORT  (GPIOC) // PC
#define BSTR_BTN_SWS_PORT   (GPIOD) // PD

// pwm timer modules for RGB leds ------------------------------

#define BSTR_LED_RED_PWM   (PWM1) // PF3
#define BSTR_LED_GREEN_PWM (1U << 3) // PB3
#define BSTR_LED_BLUE_PWM  (1U << 4) // PC4

// LCD SPI definitions. --------------------------------------------
// Connected pins in physical for LCD display (SSI2)
// J1.7 LCD SPI clock (SPI)              {TM4C123 PB4}
// J2.13 LCD SPI CS (SPI)                {TM4C123 PA4}
// J2.15 LCD SPI data (SPI)              {TM4C123 PB7}
// J2.17 LCD !RST (digital)              {TM4C123 PF0}
// J4.31 LCD RS (digital)                {TM4C123 PF4}
//
// PORT definitions
#define BP_SPI_CLK_PORT       (GPIOB)
#define BP_SPI_CS_PORT        (GPIOA)
#define BP_SPI_DATA_PORT      (GPIOB)
#define BP_LCD_RST_PORT       (GPIOF)
#define BP_LCD_RS_PORT        (GPIOF)

// pin definitions
#define BP_SPI_CLK_PIN        (1U << 4) // PB4  
#define BP_SPI_CS_PIN         (1U << 4) // PA4  
#define BP_SPI_DATA_PIN       (1U << 7) // PB7  
#define BP_LCD_RST_PIN        (1U << 0) // PF0  
#define BP_LCD_RS_PIN         (1U << 4) // PF4  

//-------------------------------------------------------------------

//--------------------------------------------------
// Joystick
// J1.5 joystick Select button (digital) {TM4C123 PE4}
// J1.2 joystick horizontal (X) (analog) {TM4C123 PB5}
// J3.26 joystick vertical (Y) (analog)  {TM4C123 PD3}
//
//--------------------------------------------------

// port definitions
//
#define BSP_BP_JST_BTN_PORT    (GPIOE) // PE4
#define BSP_BP_ADC_X_PORT      (GPIOB) // PB5
#define BSP_BP_ADC_Y_PORT      (GPIOD) // PD3


// pin definitions
//
#define BSP_BP_JST_BTN_PIN    (1U << 4) // PE4
#define BSP_BP_ADC_X_PIN      (1U << 5) // PB5
#define BSP_BP_ADC_Y_PIN      (1U << 3) // PD3

//---------------------------------------------------------------
//--------------------------------------------------
// accelerometer
// J3.23 accelerometer X (analog)        {TM4C123 PD0}
// J3.24 accelerometer Y (analog)        {TM4C123 PD1}
// J3.25 accelerometer Z (analog)        {TM4C123 PD2}
//--------------------------------------------------

// port definitions
// 
#define BSP_BP_ACC_ADC_PORT      (GPIOD) // PD2-0

// pin definitions
// 
#define BSP_BP_ACC_X_ADC_PORT    (1U << 0) // PD0
#define BSP_BP_ACC_Y_ADC_PORT    (1U << 1) // PD1
#define BSP_BP_ACC_Z_ADC_PORT    (1U << 2) // PD2

//--------------------------------------------------------------


#endif /* PORTS_H */
