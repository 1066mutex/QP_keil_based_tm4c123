/*****************************************************************************
* Product: DPP example, FreeRTOS kernel
* Last Updated for Version: 5.4.0
* Date of the Last Update:  2015-03-07
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) Quantum Leaps, LLC. state-machine.com.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* Contact information:
* Web  : http://www.state-machine.com
* Email: info@state-machine.com
*****************************************************************************/
#ifndef BSP_H
#define BSP_H
#include <stdint.h>
//!SYSTEM CORE CLOCK == 50MHZ
#define BSP_TICKS_PER_SEC    configTICK_RATE_HZ

/* 1 If booster pack is used in project */
#define BOOSTER_PACK_USED      1


void BSP_init(void);
void BSP_displayPaused(uint8_t paused);
void BSP_displayPhilStat(uint8_t n, char_t const *stat);
void BSP_terminate(int16_t result);

void BSP_randomSeed(uint32_t seed);   /* random seed */
uint32_t BSP_random(void);            /* pseudo-random generator */

// ------------BSP_Delay1ms------------
// Simple delay function which delays about n
// milliseconds.
// Inputs: n  number of 1 msec to wait
// Outputs: none
void BSP_Delay1ms(uint32_t n);

// LEDs on tm4c123
//
void BSP_ledRedOn(void);
void BSP_ledRedOff(void);
void BSP_ledBlueOn(void);
void BSP_ledBlueOff(void);
void BSP_ledGreenOn(void);
void BSP_ledGreenOff(void);

/* LEDs on Booster pack */
void BSP_BP_D_LedsInit(void);
void BSP_BP_RGB_ledsInit(void);
void BSP_BP_LedRedDuty(uint8_t dutyCycle);
void BSP_BP_LedBlueDuty(uint8_t dutyCycle);
void BSP_BP_LedGreenDuty(uint8_t dutyCycle);
void BSP_BP_LedRedOn(void);
void BSP_BP_LedRedOff(void);
void BSP_BP_LedBlueOn(void);
void BSP_BP_LedBlueOff(void);
void BSP_BP_LedGreenOn(void);
void BSP_BP_LedGreenOff(void);

/*booster pack buzzer*/
void BSP_BP_BuzzerIO_Init(uint16_t duty);
void BSP_BP_Buzzer_Set(uint16_t duty);
void BSP_BP_Buzzer_Freq(float32_t freq);

/* Booster pack I/O initialisation */
void BSP_bstrPackBtnsInit(void);  // buttons initialisation.
void BSP_bstrPackLedsDigitalInit(void); 

/* Booster TFT SPI  */
void BSP_BP_SPI_TFT_Init(const uint8_t *cmdList);
void BSP_BP_TFT_CS_LOW(void);
void BSP_BP_TFT_CS_HIGH(void);

/* Booster TFT REST  */
void BSP_BP_TFT_RESET_HIGH(void);
void BSP_BP_TFT_RESET_LOW(void);
// This is a helper function that sends a piece of 8-bit data to the LCD.
// Inputs: c  8-bit data to transmit
// Outputs: 8-bit reply
// Assumes: SSI2 and ports have already been initialized and enabled
uint8_t BSP_BP_SPIwritedata(uint8_t c);
// This is a helper function that sends an 8-bit command to the LCD.
// Inputs: c  8-bit code to transmit
// Outputs: 8-bit reply
// Assumes: SSI2 and ports have already been initialized and enabled
uint8_t BSP_BP_SPIwritecommand(uint8_t c);



#endif /* BSP_H */
