/*
 * @File: port.h
 * @Author: eddie.mhako 
 * @Date: 2021-06-24 20:26:35 
 * @Last Modified by: eddie.mhako
 * @Last Modified time: 2021-08-15 11:47:51
 * 
 * These are the port definitions for the tm4c123 and 
 * MKII BOOSER PACK attached. 
 */

//  J1   J3               J4   J2
// [ 1] [21]             [40] [20]
// [ 2] [22]             [39] [19]
// [ 3] [23]             [38] [18]
// [ 4] [24]             [37] [17]
// [ 5] [25]             [36] [16]
// [ 6] [26]             [35] [15]
// [ 7] [27]             [34] [14]
// [ 8] [28]             [33] [13]
// [ 9] [29]             [32] [12]
// [10] [30]             [31] [11]

// Connected pins in physical order
// J1.1 +3.3V (power)
// J1.2 joystick horizontal (X) (analog) {TM4C123 PB5/AIN11}
// J1.3 UART from Bluetooth to LaunchPad (UART) {TM4C123 PB0}
// J1.4 UART from LaunchPad to Bluetooth (UART) {TM4C123 PB1}
// J1.5 joystick Select button (digital) {TM4C123 PE4}
// J1.6 microphone (analog)              {TM4C123 PE5/AIN8}
// J1.7 LCD SPI clock (SPI)              {TM4C123 PB4}
// J1.8 ambient light (OPT3001) interrupt (digital) {TM4C123 PA5}
// J1.9 ambient light (OPT3001) and temperature sensor (TMP006) I2C SCL (I2C)  {TM4C123 PA6}
// J1.10 ambient light (OPT3001) and temperature sensor (TMP006) I2C SDA (I2C) {TM4C123 PA7}
//--------------------------------------------------
// J2.11 temperature sensor (TMP006) interrupt (digital) {TM4C123 PA2}
// J2.12 nothing                         {TM4C123 PA3}
// J2.13 LCD SPI CS (SPI)                {TM4C123 PA4}
// J2.14 nothing                         {TM4C123 PB6}
// J2.15 LCD SPI data (SPI)              {TM4C123 PB7}
// J2.16 nothing (reset)
// J2.17 LCD !RST (digital)              {TM4C123 PF0}
// J2.18 Profile 4                       {TM4C123 PE0}
// J2.19 servo PWM                       {TM4C123 PB2}
// J2.20 GND (ground)
//--------------------------------------------------
// J3.21 +5V (power)
// J3.22 GND (ground)
// J3.23 accelerometer X (analog)        {TM4C123 PD0/AIN7}
// J3.24 accelerometer Y (analog)        {TM4C123 PD1/AIN6}
// J3.25 accelerometer Z (analog)        {TM4C123 PD2/AIN5}
// J3.26 joystick vertical (Y) (analog)  {TM4C123 PD3/AIN4}
// J3.27 Profile 0                       {TM4C123 PE1}
// J3.28 Profile 1                       {TM4C123 PE2}
// J3.29 Profile 2                       {TM4C123 PE3}
// J3.30 Profile 3                       {TM4C123 PF1}
//--------------------------------------------------
// J4.31 LCD RS (digital)                {TM4C123 PF4}
// J4.32 user Button2 (bottom) (digital) {TM4C123 PD7}
// J4.33 user Button1 (top) (digital)    {TM4C123 PD6}
// J4.34 Profile 6/gator hole switch     {TM4C123 PC7}
// J4.35 nothing                         {TM4C123 PC6}
// J4.36 Profile 5                       {TM4C123 PC5}
// J4.37 RGB LED blue (PWM)              {TM4C123 PC4}
// J4.38 RGB LED green (PWM)             {TM4C123 PB3}
// J4.39 RGB LED red (jumper up) or LCD backlight (jumper down) (PWM) {TM4C123 PF3}
// J4.40 buzzer (PWM)                    {TM4C123 PF2}
//--------------------------------------------------
// Connected pins in logic order
// power and reset
// J1.1 +3.3V (power)
// J3.21 +5V (power)
// J3.22 GND (ground)
// J2.20 GND (ground)
// J2.16 nothing (reset)
//--------------------------------------------------
// LCD graphics
// J1.7 LCD SPI clock (SPI)              {TM4C123 PB4}
// J2.13 LCD SPI CS (SPI)                {TM4C123 PA4}
// J2.15 LCD SPI data (SPI)              {TM4C123 PB7}
// J2.17 LCD !RST (digital)              {TM4C123 PF0}
// J4.31 LCD RS (digital)                {TM4C123 PF4}
//--------------------------------------------------
// 3-color LED
// J4.37 RGB LED blue (PWM)              {TM4C123 PC4}
// J4.38 RGB LED green (PWM)             {TM4C123 PB3}
// J4.39 RGB LED red (jumper up) or LCD backlight (jumper down) (PWM) {TM4C123 PF3}
//--------------------------------------------------
// user buttons
// J4.32 user Button2 (bottom) (digital) {TM4C123 PD7}
// J4.33 user Button1 (top) (digital)    {TM4C123 PD6}
//--------------------------------------------------
// buzzer output
// J4.40 buzzer (PWM)                    {TM4C123 PF2}
//--------------------------------------------------
// Joystick
// J1.5 joystick Select button (digital) {TM4C123 PE4}
// J1.2 joystick horizontal (X) (analog) {TM4C123 PB5/AIN11}
// J3.26 joystick vertical (Y) (analog)  {TM4C123 PD3/AIN4}
//--------------------------------------------------
// accelerometer
// J3.23 accelerometer X (analog)        {TM4C123 PD0/AIN7}
// J3.24 accelerometer Y (analog)        {TM4C123 PD1/AIN6}
// J3.25 accelerometer Z (analog)        {TM4C123 PD2/AIN5}
//--------------------------------------------------
// microphone
// J1.6 microphone (analog)              {TM4C123 PE5/AIN8}
//--------------------------------------------------
// light and temperature sensors (I2C)
// J1.8 ambient light (OPT3001) interrupt (digital) {TM4C123 PA5}
// J1.9 ambient light (OPT3001) and temperature sensor (TMP006) I2C SCL (I2C)  {TM4C123 PA6}
// J1.10 ambient light (OPT3001) and temperature sensor (TMP006) I2C SDA (I2C) {TM4C123 PA7}
// J2.11 temperature sensor (TMP006) interrupt (digital) {TM4C123 PA2}
//--------------------------------------------------
// Bluetooth booster
// J1.3 UART from Bluetooth to LaunchPad (UART) {TM4C123 PB0}
// J1.4 UART from LaunchPad to Bluetooth (UART) {TM4C123 PB1}
//--------------------------------------------------
// profile pins
// J3.27 Profile 0                       {TM4C123 PE1}
// J3.28 Profile 1                       {TM4C123 PE2}
// J3.29 Profile 2                       {TM4C123 PE3}
// J3.30 Profile 3                       {TM4C123 PF1}
// J2.18 Profile 4                       {TM4C123 PE0}
// J4.36 Profile 5                       {TM4C123 PC5}
// J4.34 Profile 6                       {TM4C123 PC7}
//--------------------------------------------------
// unconnected pins
// J2.12 nothing                         {TM4C123 PA3}
// J2.14 nothing                         {TM4C123 PB6}
// J2.19 servo PWM                       {TM4C123 PB2}
// J4.35 nothing                         {TM4C123 PC6}

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
// todo: update bsp.
#define BSP_BP_ACC_X_ADC_PIN    (1U << 0) // PD0
#define BSP_BP_ACC_Y_ADC_PIN    (1U << 1) // PD1
#define BSP_BP_ACC_Z_ADC_PIN    (1U << 2) // PD2

//--------------------------------------------------------------
//
// OPT3001 ambient light sensor
// J1.8  interrupt (digital) {TM4C123 PA5}
// J1.9  I2C SCL (I2C)       {TM4C123 PA6}
// J1.10 I2C SDA (I2C)       {TM4C123 PA7}

// port definitions
//
//todo: update bsp.
#define BSP_BP_OPT3001_IRQ_PORT        (GPIOA) // PA5
#define BSP_BP_OPT3001_I2C_SCL_PORT    (GPIOA) // PA6
#define BSP_BP_OPT3001_I2C_SDA_PORT    (GPIOA) // PA7

// pin definitions
//
#define BSP_BP_OPT3001_IRQ_PIN        (1U << 5) // PA5
#define BSP_BP_OPT3001_I2C_SCL_PIN    (1U << 6) // PA6
#define BSP_BP_OPT3001_I2C_SDA_PIN    (1U << 7) // PA7

//--------------------------------------------------

#endif /* PORTS_H */
