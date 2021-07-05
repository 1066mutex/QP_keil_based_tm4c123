
// bsp_bstPack.h
// Runs on either the TM4C123 or STM32F4 with an Educational BoosterPack MKII
// (BOOSTXL-EDUMKII) This file contains the function prototypes for the software
// interface to the MKII BoosterPack. This board support package (BSP) is an
// abstraction layer,

// pins layout
//
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
//
// J1.1 +3.3V (power)
// J1.2 joystick horizontal (X) (analog) {TM4C123 PB5/AIN11}
// J1.3 UART from Bluetooth to LaunchPad (UART) {TM4C123 PB0}
// J1.4 UART from LaunchPad to Bluetooth (UART) {TM4C123 PB1}
// J1.5 joystick Select button (digital) {TM4C123 PE4}
// J1.6 microphone (analog)              {TM4C123 PE5}
// J1.7 LCD SPI clock (SPI)              {TM4C123 PB4}
// J1.8 ambient light (OPT3001) interrupt (digital) {TM4C123 PA5}
// J1.9 ambient light (OPT3001) and temperature sensor (TMP006) I2C SCL (I2C)
//      {TM4C123 PA6}
// J1.10 ambient light (OPT3001) and temperature
//       sensor (TMP006) I2C SDA (I2C) {TM4C123 PA7}
// TODO: add I/O used for the STM32 board.
// -------------------------------------------------------
//
// J2.11 temperature sensor (TMP006) interrupt (digital) {TM4C123 PA2}
// J2.12 nothing                         {TM4C123 PA3, }
// J2.13 LCD SPI CS (SPI)                {TM4C123 PA4, }
// J2.14 nothing                         {TM4C123 PB6, }
// J2.15 LCD SPI data (SPI)              {TM4C123 PB7, }
// J2.16 nothing (reset)
// J2.17 LCD !RST (digital)              {TM4C123 PF0, }
// J2.18 Profile 4                       {TM4C123 PE0, }
// J2.19 servo PWM                       {TM4C123 PB2, }
// J2.20 GND (ground)
//--------------------------------------------------
//
// J3.21 +5V (power)
// J3.22 GND (ground)
// J3.23 accelerometer X (analog)        {TM4C123 PD0/AIN7, }
// J3.24 accelerometer Y (analog)        {TM4C123 PD1/AIN6, }
// J3.25 accelerometer Z (analog)        {TM4C123 PD2/AIN5, }
// J3.26 joystick vertical (Y) (analog)  {TM4C123 PD3/AIN4, }
// J3.27 Profile 0                       {TM4C123 PE1, }
// J3.28 Profile 1                       {TM4C123 PE2, }
// J3.29 Profile 2                       {TM4C123 PE3, }
// J3.30 Profile 3                       {TM4C123 PF1, }
//--------------------------------------------------
//
// J4.31 LCD RS (digital)                {TM4C123 PF4, }
// J4.32 user Button2 (bottom) (digital) {TM4C123 PD7, }
// J4.33 user Button1 (top) (digital)    {TM4C123 PD6, }
// J4.34 Profile 6/gator hole switch     {TM4C123 PC7, }
// J4.35 nothing                         {TM4C123 PC6, }
// J4.36 Profile 5                       {TM4C123 PC5, }
// J4.37 RGB LED blue (PWM)              {TM4C123 PC4, }
// J4.38 RGB LED green (PWM)             {TM4C123 PB3, }
// J4.39 RGB LED red (jumper up) or LCD backlight
//       (jumper down) (PWM)             {TM4C123 PF3, }
// J4.40 buzzer (PWM)                    {TM4C123 PF2, }
//--------------------------------------------------
//
// Connected pins in logic order
// power and reset
// J1.1 +3.3V (power)
// J3.21 +5V (power)
// J3.22 GND (ground)
// J2.20 GND (ground)
// J2.16 nothing (reset)
//--------------------------------------------------
//
// LCD graphics
// J1.7 LCD SPI clock (SPI)              {TM4C123 PB4, }
// J2.13 LCD SPI CS (SPI)                {TM4C123 PA4, }
// J2.15 LCD SPI data (SPI)              {TM4C123 PB7, }
// J2.17 LCD !RST (digital)              {TM4C123 PF0, }
// J4.31 LCD RS (digital)                {TM4C123 PF4, }
//--------------------------------------------------
//
// 3-color LED
// J4.37 RGB LED blue (PWM)              {TM4C123 PC4, }
// J4.38 RGB LED green (PWM)             {TM4C123 PB3, }
// J4.39 RGB LED red (jumper up) or LCD backlight
//      (jumper down) (PWM)              {TM4C123PF3, }
//--------------------------------------------------
//
// user buttons
// J4.32 user Button2 (bottom) (digital) {TM4C123 PD7, }
// J4.33 user Button1 (top) (digital)    {TM4C123 PD6, }
//--------------------------------------------------
//
// buzzer output
// J4.40 buzzer (PWM)                    {TM4C123 PF2, }
//--------------------------------------------------
//
// Joystick
// J1.5 joystick Select button (digital) {TM4C123 PE4, }
// J1.2 joystick horizontal (X) (analog) {TM4C123 PB5/AIN11, }
// J3.26 joystick vertical (Y) (analog)  {TM4C123 PD3/AIN4, }
//--------------------------------------------------
//
// accelerometer
// J3.23 accelerometer X (analog)        {TM4C123 PD0/AIN7, }
// J3.24 accelerometer Y (analog)        {TM4C123 PD1/AIN6, }
// J3.25 accelerometer Z (analog)        {TM4C123 PD2/AIN5, }
//--------------------------------------------------
// microphone
// J1.6 microphone (analog)              {TM4C123 PE5/AIN8, }
//--------------------------------------------------
//
// light and temperature sensors (I2C)
// J1.8 ambient light (OPT3001) interrupt (digital) {TM4C123 PA5, }
// J1.9 ambient light (OPT3001) and temperature
//      sensor (TMP006) I2C SCL (I2C)               {TM4C123 PA6, }
// J1.10 ambient light (OPT3001) and temperature sensor
//       (TMP006) I2C SDA (I2C)                     {TM4C123 PA7, }
// J2.11 temperature sensor (TMP006) interrupt (digital) {TM4C123 PA2, }
//--------------------------------------------------
//
// Bluetooth booster
// J1.3 UART from Bluetooth to LaunchPad (UART) {TM4C123 PB0, MSP432 P3.2}
// J1.4 UART from LaunchPad to Bluetooth (UART) {TM4C123 PB1, MSP432 P3.3}
//--------------------------------------------------
//
// profile pins
// J3.27 Profile 0                       {TM4C123 PE1, MSP432 P4.5}
// J3.28 Profile 1                       {TM4C123 PE2, MSP432 P4.7}
// J3.29 Profile 2                       {TM4C123 PE3, MSP432 P5.4}
// J3.30 Profile 3                       {TM4C123 PF1, MSP432 P5.5}
// J2.18 Profile 4                       {TM4C123 PE0, MSP432 P3.0}
// J4.36 Profile 5                       {TM4C123 PC5, MSP432 P6.6}
// J4.34 Profile 6                       {TM4C123 PC7, MSP432 P2.3}
//--------------------------------------------------
//
// unconnected pins
// J2.12 nothing                         {TM4C123 PA3, MSP432 P5.2}
// J2.14 nothing                         {TM4C123 PB6, MSP432 P1.7}
// J2.19 servo PWM                       {TM4C123 PB2, MSP432 P2.5}
// J4.35 nothing                         {TM4C123 PC6, MSP432 P6.7}

#ifndef BSP_BSTPACK_H
#define BSP_BSTPACK_H

/* !!! device initialisation ##########################################################:

1)
pass in an object that has parameters for which sensors/peripherals have been
used within the project. initialise only availabe functions.
2)
Use #define to configure which features are used, see FreeRTOSConfig.h for example.
then use if() to initialise the features. 

 */

//! use TDD
/* TODO: buttons initialisation function for both buttons 
         calls the main bsp for mcu board. 
*/

/* TODO: buttons state function for both buttons 
         NOTE: not sure if this is call main bsp function for I/O 
               as we will be using message passing. 
*/
/* TODO: joystick initialisation and input functions */

/* TODO: RGB led as PWM initialisation functions */


/* TODO: RGB led as PWM set RGB values functions */

/* TODO: RGB led as digital (on/off) initialisation functions */
/* TODO: RGB led set/reset/toggle function use enums 
         Can use OOP single function to handle all features 
*/

/* TODO: buzzer init, and set tones(pwm) */
/* TODO: accelerometer init, and out put. */

/* ################################################################################# */

/* LCD display ###################################################################### */





#endif /* BSP_BSTPACK_H */
