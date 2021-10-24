/*****************************************************************************
* Product: DPP example, EK-TM4C123GXL board, FreeRTOS kernel
* Last Updated for Version: 6.0.4
* Date of the Last Update:  2018-01-10
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) Quantum Leaps, LLC. All rights reserved.
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
* https://state-machine.com
* mailto:info@state-machine.com
*****************************************************************************/
#include "qpc.h"
#include "defs.h"
#include "bsp.h"
#include "logger.h"
#include "bsp_bp.h"
#include "ports.h"
#include "profile.h"
#include <stdint.h>
//#include "arm_math.h"

#include "TM4C123GH6PM.h" /* the device specific header (TI) */
#include "UART0.h"        //! used for testing uart device
#include "rom.h"    /* the built-in ROM functions (TI) */
#include "sysctl.h" /* system control driver (TI) */
#include "gpio.h" /* GPIO driver (TI) */
#include "hw_i2c.h"

/* add other drivers if necessary... */

#define TIMER_1A_PSV (TIMER1->TAPV)                    // TIMER prescaler value.
#define TIMER_1A_CLK (SystemCoreClock / TIMER_1A_PSV)  // timer 1A clk

#define PWM_CYCLES (SystemCoreClock / 2048)  // time reload value for RGB (pwm) leds.

#define DELAY 0x80  //  booster pack LCD display delay

// todo: move to bsp.h all uart stuff
#define UART_FR_TXFE 0x00000080      // UART Transmit FIFO Empty
#define UART_FR_RXFF 0x00000040      // UART Receive FIFO Full
#define UART_FR_TXFF 0x00000020      // UART Transmit FIFO Full
#define UART_FR_RXFE 0x00000010      // UART Receive FIFO Empty
#define UART_FR_BUSY 0x00000008      // UART Transmit Busy
#define UART_LCRH_WLEN_8 0x00000060  // 8 bit word length
#define UART_LCRH_FEN 0x00000010     // UART Enable FIFOs
#define UART_CTL_UARTEN 0x00000001   // UART Enable
#define UART_IFLS_RX1_8 0x00000000   // RX FIFO >= 1/8 full
#define UART_IFLS_TX1_8 0x00000000   // TX FIFO <= 1/8 full
#define UART_IM_RTIM 0x00000040      // UART Receive Time-Out Interrupt 
                                     // Mask
#define UART_IM_TXIM 0x00000020      // UART Transmit Interrupt Mask
#define UART_IM_RXIM 0x00000010      // UART Receive Interrupt Mask
#define UART_RIS_RTRIS 0x00000040    // UART Receive Time-Out Raw 
                                     // Interrupt Status
#define UART_RIS_TXRIS 0x00000020    // UART Transmit Raw Interrupt 
                                     // Status
#define UART_RIS_RXRIS 0x00000010    // UART Receive Raw Interrupt 
                                     // Status
#define UART_ICR_RTIC 0x00000040     // Receive Time-Out Interrupt Clear
#define UART_ICR_TXIC 0x00000020     // Transmit Interrupt Clear
#define UART_ICR_RXIC 0x00000010     // Receive Interrupt Clear

Q_DEFINE_THIS_FILE /* define the name of this file for assertions */

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
* Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
* DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
*/
    enum KernelUnawareISRs { /* see NOTE1 */
                             // UART0_PRIO,
                             /* ... */
                             MAX_KERNEL_UNAWARE_CMSIS_PRI /* keep always last */
    };
/* "kernel-unaware" interrupts can't overlap "kernel-aware" interrupts */
Q_ASSERT_COMPILE(
    MAX_KERNEL_UNAWARE_CMSIS_PRI <= (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)));

enum KernelAwareISRs {
    SYSTICK_PRIO = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)),  //5
    GPIOA_PRIO = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)),    //5
    UART1_PRIO = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)),    //5
    UART0_PRIO = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)),
    ADC1SS3_PRIO, /* ADC0 SS3 priority */

    /* ... */
    MAX_KERNEL_AWARE_CMSIS_PRI /* keep always last */

};
/* "kernel-aware" interrupts should not overlap the PendSV priority */
Q_ASSERT_COMPILE(MAX_KERNEL_AWARE_CMSIS_PRI <= (0xFF >> (8 - __NVIC_PRIO_BITS)));

/* Local-scope objects ------------------------------------------------------*/
static uint32_t l_rnd;  // random seed

/* Local-scope functions ------------------------------------------------------*/

static void l_switchDebounce(void);  // debounces all system switchies.
//static void mcuTempSensorInit(void);
static void l_adcinit1(void);

#ifdef Q_SPY

QSTimeCtr QS_tickTime_;
QSTimeCtr QS_tickPeriod_;

/* QS identifiers for non-QP sources of events */
static uint8_t const l_TickHook = (uint8_t)0;
static uint8_t const l_GPIOPortA_IRQHandler = (uint8_t)0;

#define UART_BAUD_RATE 115200U
#define UART_FR_TXFE (1U << 7)
#define UART_FR_RXFE (1U << 4)
#define UART_TXFIFO_DEPTH 16U

enum AppRecords { /* application-specific trace records */
                  PHILO_STAT = QS_USER,
                  PAUSED_STAT,
                  COMMAND_STAT
};

#endif
/* Local-scope functions ------------------------------------------------------*/
/* ISRs used in this project ===============================================*/

/* NOTE: this ISR is for testing of the various preemption scenarios
*  by triggering the GPIOPortA interrupt from the debugger. You achieve
*  this by writing 0 to the  SWTRIG register at 0xE000,EF00.
*
*  Code Composer Studio: From the CCS debugger you need open the register
*  window and select NVIC registers from the drop-down list. You scroll to
*  the NVIC_SW_TRIG register, which denotes the Software Trigger Interrupt
*  Register in the NVIC. To trigger the GPIOA interrupt you need to write
*  0x00 to the NVIC_SW_TRIG by clicking on this field, entering the value,
*  and pressing the Enter key.
*
*  IAR EWARM: From the C-Spy debugger you need to open Registers view and
*  select the "Other Systems Register" group. From there, you need to write
*  0 to the STIR write-only register and press enter.
*/
/* NOTE: only the "FromISR" FreeRTOS API variants are allowed in the ISRs! */

/* ISR prototypes*/
void UART1_IRQHandler(void);
void ADC1Seq3_IRQHandler(void);

/**
 * @brief UART1 ISR handler.
 * collects all received characters from the Uart FIFO buffer into a  
 * single event object and then post it to the BLE handler active object 
 * for processing.
 * NOTE:
 * at least one of two things has happened:
 * hardware RX FIFO goes from 1 to 2 or more items
 * UART receiver has timed out
 */
void UART1_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t status = UART1->RIS;  // get the raw interrupt status
    UART1->ICR = status;           // clear the asserted interrupts
    UartEvt *evt = Q_NEW_FROM_ISR(UartEvt, NEW_UART_DATA_SIG);

    evt->len = 0;
    while (((UART1->FR & UART_FR_RXFE) == 0) && (evt->len < UART1_FIFO_LEN)) {
        evt->chars[evt->len] = UART1_CHAR;
        ++evt->len;
    }
    Q_ASSERT(evt->len < UART1_FIFO_LEN);  // did'nt get  all the data.

    QACTIVE_POST_FROM_ISR(AO_Bluetooth,
                          &evt->super,
                          &xHigherPriorityTaskWoken,
                          &UART1_IRQHandler);

    /* let FreeRTOS determine if context switch is needed...  */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

// todo: remove
// /**
//  * ADC1Seq3 ISR handler.
//  * collects MKII microphone data from ADC into an event object. 
//  * then post it to the microphone handler active object 
//  * for processing.
//  */
// void ADC1Seq3_IRQHandler(void) {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//     while ((ADC1->RIS & (1U << 3)) == 0) {
//     };                                                                 // 2) wait for conversion done
//     RawRawSoundEvt *reading = Q_NEW_FROM_ISR(RawSoundEvt, NEW_SOUND_DATA_SIG);  // TODO: update all files using old NEW_TEMP_DATA_SIG

//     reading->sound = (ADC1->SSFIFO3);  //
//     ADC1->ISC |= (1U << 3);            // clear the asserted interrupts
//     QACTIVE_POST_FROM_ISR(AO_Heartbeat,
//                           (QEvt *)reading,
//                           &xHigherPriorityTaskWoken,
//                           &ADCSeq3_IRQHandler);

//     /* the usual end of FreeRTOS ISR... */
//     portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
// }



/**
 * @brief adcIsr typedef used by ADC1 ISR for microphone sound processing.
 * 
 */
typedef struct {
    int16_t soundBuff[SAMPLE_LENGTH];
    uint16_t idx;
    enum state { INDEX_0,
                 PACKING } state;
} adcIsr;
adcIsr adcData;


/**
 * @brief ADC1Seq3_IRQHandler
 * collects MKII microphone data from ADC and packs it into an event object buffer.
 * then post it to the microphone handler active object for processing
 */
void ADC1Seq3_IRQHandler(void) {
    
    adcIsr *adcDataPtr = &adcData;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t status = ADC1->RIS; // get the raw interrupt status 
    while ((ADC1->RIS & (1U << 3)) == 0) {};  // wait for conversion
    
    ADC1->ISC = status;          // clear the asserted interrupts 
    switch (adcDataPtr->state) {
        case INDEX_0: {
            // pack the first data
            //
            adcDataPtr->soundBuff[adcDataPtr->idx++] = ((uint16_t)ADC1->SSFIFO3) - (uint16_t)(2048);  // (>> 1) divide by 2
            adcDataPtr->state = PACKING;
            break;
        }
        case PACKING: { // pack the rest of the data
            if (adcDataPtr->idx != SAMPLE_LENGTH) {
                // pack the all the data
                //
                adcDataPtr->soundBuff[adcDataPtr->idx++] = ((uint16_t)ADC1->SSFIFO3) - (uint16_t)(2048);

            } else if (adcDataPtr->idx == SAMPLE_LENGTH) {
                RawSoundEvt *micData = Q_NEW_FROM_ISR(RawSoundEvt, NEW_SOUND_DATA_SIG);
                for (int i = 0; i < SAMPLE_LENGTH; i++) {
                    micData->rawSoundBuf[i] = adcDataPtr->soundBuff[i];
                }
                adcDataPtr->idx = 0;
                adcDataPtr->state = INDEX_0;
                // packing is finised, post data to AO
                // 
                QACTIVE_POST_FROM_ISR(AO_soundSensor,
                                      (QEvt *)micData,
                                      &xHigherPriorityTaskWoken,
                                      &ADCSeq3_IRQHandler);
            }

            break;
        }

        default:
            break;
    }

    /* let FreeRTOS determine if context switch is needed...  */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
   
}

#ifdef Q_SPY
/*
* ISR for receiving bytes from the QSPY Back-End
* NOTE: This ISR is "kernel-unaware" meaning that it does not interact with
* the FreeRTOS or QP and is not disabled. Such ISRs don't need to call
* portEND_SWITCHING_ISR(() at the end, but they also cannot call any
* FreeRTOS or QP APIs.
*/
void UART0_IRQHandler(void) {
    uint32_t status = UART0->RIS; /* get the raw interrupt status */
    UART0->ICR = status;          /* clear the asserted interrupts */

    while ((UART0->FR & UART_FR_RXFE) == 0) { /* while RX FIFO NOT empty */
        uint32_t b = UART0->DR;
        QS_RX_PUT(b);
    }
}
#endif
//! used to inform RTOS of a thread switch request.
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

/* Application hooks used in this project ==================================*/
/* NOTE: only the "FromISR" API variants are allowed in vApplicationTickHook */
void vApplicationTickHook(void) {  //! called by freeRtos tick()

    /* process time events for rate 0 */
    QF_TICK_X_FROM_ISR(0U, &xHigherPriorityTaskWoken, &l_TickHook);

#ifdef Q_SPY
    {
        tmp = SysTick->CTRL;            /* clear SysTick_CTRL_COUNTFLAG */
        QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
    }
#endif

    /* Debounce system switches*/
    l_switchDebounce();

    /* notify FreeRTOS to perform context switch from ISR, if needed */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
/*..........................................................................*/
void vApplicationIdleHook(void) {
    float volatile x;

    /* toggle the User LED on and then off, see NOTE01 */
    QF_INT_DISABLE();
    GPIOF->DATA_Bits[LED_BLUE] = 0xFFU; /* turn the Blue LED on  */
    GPIOF->DATA_Bits[LED_BLUE] = 0U;    /* turn the Blue LED off */
    QF_INT_ENABLE();

    /* Some flating point code is to exercise the VFP... */
    x = 1.73205F;
    x = x * 1.73205F;

#ifdef Q_SPY
    QS_rxParse(); /* parse all the received bytes */

    if ((UART0->FR & UART_FR_TXFE) != 0U) { /* TX done? */
        uint16_t fifo = UART_TXFIFO_DEPTH;  /* max bytes we can accept */
        uint8_t const *block;

        QF_INT_DISABLE();
        block = QS_getBlock(&fifo); /* try to get next block to transmit */
        QF_INT_ENABLE();

        while (fifo-- != 0) {     /* any bytes in the block? */
            UART0->DR = *block++; /* put into the FIFO */
        }
    }
#elif defined NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M3 MCU.
    */
    __WFI(); /* Wait-For-Interrupt */
#endif
}
/*..........................................................................*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    Q_ERROR();
}
/*..........................................................................*/
/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must
* provide an implementation of vApplicationGetIdleTaskMemory() to provide
* the memory that is used by the Idle task.
*/
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    /* If the buffers to be provided to the Idle task are declared inside
    * this function then they must be declared static - otherwise they will
    * be allocated on the stack and so not exists after this function exits.
    */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* Pass out a pointer to the StaticTask_t structure in which the
    * Idle task's state will be stored.
    */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    * Note that, as the array is necessarily of type StackType_t,
    * configMINIMAL_STACK_SIZE is specified in words, not bytes.
    */
    *pulIdleTaskStackSize = Q_DIM(uxIdleTaskStack);
}

/* BSP functions ===========================================================*/
void BSP_init(void) {
    /* NOTE: SystemInit() has been already called from the startup code
    *  but SystemCoreClock needs to be updated
    */
    SystemCoreClockUpdate();

    /* NOTE: The VFP (hardware Floating Point) unit is configured by FreeRTOS */

    /* enable clock for to the peripherals used by this application... */
    /* enable Run mode for GPIOD, GPIOF */
    SYSCTL->RCGCGPIO |= PORTD | PORTF | PORTB;

    /* configure the LEDs and push buttons */
    LEDS_PORT->DIR |= (LED_RED | LED_GREEN | LED_BLUE); /* set as output */
    LEDS_PORT->DEN |= (LED_RED | LED_GREEN | LED_BLUE); /* digital enable */
    LEDS_PORT->DATA_Bits[LED_RED] = 0U;                 /* turn the LED off */
    LEDS_PORT->DATA_Bits[LED_GREEN] = 0U;               /* turn the LED off */
    LEDS_PORT->DATA_Bits[LED_BLUE] = 0U;                /* turn the LED off */

    /* configure the User Switches on tm4c123 board */
    LEDS_PORT->DIR &= ~(BTN_SW1 | BTN_SW2); /*  set direction: input */
    ROM_GPIOPadConfigSet(GPIOF_BASE, (BTN_SW1 | BTN_SW2),
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
   
    /* configure the input for bluetooth status pin HM-10 module*/
    BLE_STATUS_PORT->DIR &= ~(BLE_STATE_PIN); /*  set direction: input */
    ROM_GPIOPadConfigSet(GPIOB_BASE, (BLE_STATE_PIN),
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* booster pack User Switches initialization */
    /* unlock gpio port D */
    BSTR_BTN_SWS_PORT->LOCK = 0x4C4F434B;
    BSTR_BTN_SWS_PORT->CR = 0xFF;                             /* allow changes to PD7-0 */
    BSTR_BTN_SWS_PORT->DIR &= ~(BSTR_BTN_SW3 | BSTR_BTN_SW4); /*  set direction: input */

    ROM_GPIOPadConfigSet(GPIOD_BASE, (BSTR_BTN_SW3 | BSTR_BTN_SW4),
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //BSP_BP_D_LedsInit();
    BSP_BP_RGB_ledsInit();
    BSP_BP_BuzzerIO_Init(200);
    BSP_BP_Joystick_Init();
    BSP_BP_SPI_TFT_Init();
    BSP_Microphone_Init();
    BSP_Accelerometer_Init();
    UART1_Init();
    UART0_Init();
    //mcuTempSensorInit();
    //! use when profiling TODO: add ifdef guards
    Profile_Init();
    BSP_LCD_Init();  // TODO: move to bsp_init()
    BSP_randomSeed(1234U);

    /* initialize the QS software tracing... */
    if (QS_INIT((void *)0) == 0U) {
        Q_ERROR();
    }
    QS_OBJ_DICTIONARY(&l_TickHook);
    QS_OBJ_DICTIONARY(&l_GPIOPortA_IRQHandler);
    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(PAUSED_STAT);
    QS_USR_DICTIONARY(COMMAND_STAT);
}

/* LEDs tm4c123.............................................................*/
void BSP_ledRedOn(void) {
    GPIOF->DATA_Bits[LED_RED] = LED_RED;
}

/*..........................................................................*/
void BSP_ledRedOff(void) {
    GPIOF->DATA_Bits[LED_RED] = 0U;
}
/*..........................................................................*/
void BSP_ledBlueOn(void) {
    GPIOF->DATA_Bits[LED_BLUE] = LED_BLUE;
}

/*..........................................................................*/
void BSP_ledBlueOff(void) {
    GPIOF->DATA_Bits[LED_BLUE] = 0U;
}

/*..........................................................................*/
void BSP_ledGreenOn(void) {
    GPIOF->DATA_Bits[LED_GREEN] = LED_GREEN;
}

/*..........................................................................*/
void BSP_ledGreenOff(void) {
    GPIOF->DATA_Bits[LED_GREEN] = 0U;
}

/*..........................................................................*/

/* LEDs on BoosterPack ....................................................................*/

void BSP_BP_LedGreenOff(void) {
    BSTR_LED_GREEN_PORT->DATA_Bits[BSTR_LED_GREEN] = 0U;
}

/*..........................................................................*/
void BSP_BP_LedGreenOn(void) {
    BSTR_LED_GREEN_PORT->DATA_Bits[BSTR_LED_GREEN] = BSTR_LED_GREEN;
}
/*..........................................................................*/
void BSP_BP_LedRedOff(void) {
    BSTR_LED_RED_PORT->DATA_Bits[BSTR_LED_RED] = 0U;
}

/*..........................................................................*/
void BSP_BP_LedRedOn(void) {
    BSTR_LED_RED_PORT->DATA_Bits[BSTR_LED_RED] = BSTR_LED_RED;
}
/*..........................................................................*/
/*..........................................................................*/
void BSP_BP_LedBlueOff(void) {
    BSTR_LED_BLUE_PORT->DATA_Bits[BSTR_LED_BLUE] = 0U;
}
/*..........................................................................*/
void BSP_BP_LedBlueOn(void) {
    BSTR_LED_BLUE_PORT->DATA_Bits[BSTR_LED_BLUE] = BSTR_LED_BLUE;
}
/*..........................................................................*/

/* TFT spi I/O BoosterPack ..............................................................*/

void BSP_BP_TFT_CS_LOW(void) {
    BP_SPI_CS_PORT->DATA_Bits[BP_SPI_CS_PIN] = 0U;
}
//...........................................................................
void BSP_BP_TFT_CS_HIGH(void) {
    BP_SPI_CS_PORT->DATA_Bits[BP_SPI_CS_PIN] = BP_SPI_CS_PIN;
}
//...........................................................................
void BSP_BP_TFT_RESET_HIGH(void) {
    BP_LCD_RST_PORT->DATA_Bits[BP_LCD_RST_PIN] = BP_LCD_RST_PIN;
}
//...........................................................................
void BSP_BP_TFT_RESET_LOW(void) {
    BP_LCD_RST_PORT->DATA_Bits[BP_LCD_RST_PIN] = 0U;
}
//...........................................................................
void BSP_BP_TFT_DC_COMMAND(void) {
    BP_LCD_RS_PORT->DATA_Bits[BP_LCD_RS_PIN] = 0U;
}
//...........................................................................
void BSP_BP_TFT_DC_DATA(void) {
    BP_LCD_RS_PORT->DATA_Bits[BP_LCD_RS_PIN] = BP_LCD_RS_PIN;
}
//...........................................................................
/**
 * @brief BSP_BLE_SATE
 * checks the status of the pin connected to the bluetooth device.
 * 
 * @return uint8_t 
 *         1 == ble is connected to external device
 *         0 == no device connected
 */
uint8_t BSP_BLE_SATE(void) {
   return BLE_STATUS_PORT->DATA_Bits[BLE_STATE_PIN] ;
}
//...........................................................................

// TODO: will not need this but a good example on toggling leds as display.
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    GPIOF->DATA_Bits[LED_RED] =
        ((stat[0] == 'e') /* Is Philo[n] eating? */
             ? 0xFFU      /* turn the LED1 on  */
             : 0U);       /* turn the LED1 off */

    QS_BEGIN(PHILO_STAT, AO_Philo[n]) /* application-specific record begin */
    QS_U8(1, n);                      /* Philosopher number */
    QS_STR(stat);                     /* Philosopher status */
    QS_END()
}
/*..........................................................................*/
void BSP_displayPaused(uint8_t paused) {
    GPIOF->DATA_Bits[LED_RED] = ((paused != 0U) ? 0xFFU : 0U);

    QS_BEGIN(PAUSED_STAT, (void *)0) /* application-specific record begin */
    QS_U8(1, paused);                /* Paused status */
    QS_END()
}
/*..........................................................................*/
uint32_t BSP_random(void) { /* a very cheap pseudo-random-number generator */
    uint32_t rnd;

    /* exercise the FPU with some floating point computations */
    /* NOTE: this code can be only called from a task that created with
    * the option OS_TASK_OPT_SAVE_FP.
    */
    float volatile x;
    x = 3.1415926F;
    x = x + 2.7182818F;

    vTaskSuspendAll(); /* lock FreeRTOS scheduler */
    /* "Super-Duper" Linear Congruential Generator (LCG)
    * LCG(2^32, 3*7*11*13*23, 0, seed)
    */
    rnd = l_rnd * (3U * 7U * 11U * 13U * 23U);
    l_rnd = rnd;      /* set for the next time */
    xTaskResumeAll(); /* unlock the FreeRTOS scheduler */

    return (rnd >> 8);
}
/*..........................................................................*/
void BSP_randomSeed(uint32_t seed) {
    l_rnd = seed;
}
/*..........................................................................*/
void BSP_terminate(int16_t result) {
    (void)result;
}

/* QF callbacks ============================================================*/
void QF_onStartup(void) {
    /* set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate */
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    /* assing all priority bits for preemption-prio. and none to sub-prio. */
    NVIC_SetPriorityGrouping(0U);

    /* set priorities of ALL ISRs used in the system, see NOTE00
    *
    * !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    * Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    * DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    */
    NVIC_SetPriority(UART0_IRQn, UART0_PRIO);
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIO);
    NVIC_SetPriority(GPIOA_IRQn, GPIOA_PRIO);
    NVIC_SetPriority(ADC1SS3_IRQn, ADC1SS3_PRIO);
    NVIC_SetPriority(UART1_IRQn, UART1_PRIO);

    /* ... */

    /* enable IRQs... */
    NVIC_EnableIRQ(GPIOA_IRQn);
    NVIC_EnableIRQ(ADC1SS3_IRQn);
    NVIC_EnableIRQ(UART1_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);

#ifdef Q_SPY
    NVIC_EnableIRQ(UART0_IRQn); /* UART0 interrupt used for QS-RX */
#endif
}
/*..........................................................................*/
void QF_onCleanup(void) {
}

/*..........................................................................*/
void Q_onAssert(char const *module, int loc) {
    /*
    * NOTE: add here your application-specific error handling
    */
    (void)module;
    (void)loc;
    QS_ASSERTION(module, loc, (uint32_t)10000U); /* report assertion to QS */

#ifndef NDEBUG
    /* light up all LEDs */
    GPIOF->DATA_Bits[LED_GREEN | LED_RED | LED_BLUE] = 0xFFU;
    /* for debugging, hang on in an endless loop toggling the RED LED... */
    while (GPIOF->DATA_Bits[BTN_SW1] != 1) {
        GPIOF->DATA = LED_RED;
        GPIOF->DATA = 0U;
    }
#endif

    NVIC_SystemReset();
}

/* QS callbacks ============================================================*/
#ifdef Q_SPY
/*..........................................................................*/
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsTxBuf[2 * 1024]; /* buffer for QS-TX channel */
    static uint8_t qsRxBuf[100];      /* buffer for QS-RX channel */
    uint32_t tmp;

    QS_initBuf(qsTxBuf, sizeof(qsTxBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    /* enable clock for UART0 and GPIOA (used by UART0 pins) */
    SYSCTL->RCGCUART |= (1U << 0); /* enable Run mode for UART0 */
    SYSCTL->RCGCGPIO |= (1U << 0); /* enable Run mode for GPIOA */

    /* configure UART0 pins for UART operation */
    tmp = (1U << 0) | (1U << 1);
    GPIOA->DIR &= ~tmp;
    GPIOA->SLR &= ~tmp;
    GPIOA->ODR &= ~tmp;
    GPIOA->PUR &= ~tmp;
    GPIOA->PDR &= ~tmp;
    GPIOA->AMSEL &= ~tmp; /* disable analog function on the pins */
    GPIOA->AFSEL |= tmp;  /* enable ALT function on the pins */
    GPIOA->DEN |= tmp;    /* enable digital I/O on the pins */
    GPIOA->PCTL &= ~0x00U;
    GPIOA->PCTL |= 0x11U;

    /* configure the UART for the desired baud rate, 8-N-1 operation */
    tmp = (((SystemCoreClock * 8U) / UART_BAUD_RATE) + 1U) / 2U;
    UART0->IBRD = tmp / 64U;
    UART0->FBRD = tmp % 64U;
    UART0->LCRH = (0x3U << 5);  /* configure 8-N-1 operation */
    UART0->LCRH |= (0x1U << 4); /* enable FIFOs */
    UART0->CTL = (1U << 0)      /* UART enable */
                 | (1U << 8)    /* UART TX enable */
                 | (1U << 9);   /* UART RX enable */

    /* configure UART interrupts (for the RX channel) */
    UART0->IM |= (1U << 4) | (1U << 6); /* enable RX and RX-TO interrupt */
    UART0->IFLS |= (0x2U << 2);         /* interrupt on RX FIFO half-full */
    /* NOTE: do not enable the UART0 interrupt yet. Wait till QF_onStartup() */

    QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */

    /* setup the QS filters... */
    QS_FILTER_ON(QS_SM_RECORDS); /* state machine records */
    QS_FILTER_ON(QS_UA_RECORDS); /* all usedr records */
    //QS_FILTER_ON(QS_MUTEX_LOCK);
    //QS_FILTER_ON(QS_MUTEX_UNLOCK);

    return (uint8_t)1; /* return success */
}
/*..........................................................................*/
void QS_onCleanup(void) {
}
/*..........................................................................*/
QSTimeCtr QS_onGetTime(void) {                               /* NOTE: invoked with interrupts DISABLED */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { /* not set? */
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    } else { /* the rollover occured, but the SysTick_ISR did not run yet */
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
/*..........................................................................*/
void QS_onFlush(void) {
    uint16_t fifo = UART_TXFIFO_DEPTH; /* Tx FIFO depth */
    uint8_t const *block;
    while ((block = QS_getBlock(&fifo)) != (uint8_t *)0) {
        /* busy-wait as long as TX FIFO has data to transmit */
        while ((UART0->FR & UART_FR_TXFE) == 0) {
        }

        while (fifo-- != 0U) {    /* any bytes in the block? */
            UART0->DR = *block++; /* put into the TX FIFO */
        }
        fifo = UART_TXFIFO_DEPTH; /* re-load the Tx FIFO depth */
    }
}
/*..........................................................................*/
/*! callback function to reset the target (to be implemented in the BSP) */
void QS_onReset(void) {
    NVIC_SystemReset();
}
/*..........................................................................*/
/*! callback function to execute a user command (to be implemented in BSP) */
void QS_onCommand(uint8_t cmdId,
                  uint32_t param1, uint32_t param2, uint32_t param3) {
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;

    /* application-specific record */
    QS_BEGIN(COMMAND_STAT, (void *)1)
    QS_U8(2, cmdId);
    QS_U32(8, param1);
    QS_U32(8, param2);
    QS_U32(8, param3);
    QS_END()

    if (cmdId == 10U) {
        Q_ERROR(); /* for testing of assertion failure */
    } else if (cmdId == 11U) {
        assert_failed("QS_onCommand", 123);
    }
}

#endif /* Q_SPY */
/*--------------------------------------------------------------------------*/

/*****************************************************************************
* NOTE1:
* The configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY constant from the
* FreeRTOS configuration file specifies the highest ISR priority that
* is disabled by the QF framework. The value is suitable for the
* NVIC_SetPriority() CMSIS function.
*
* Only ISRs prioritized at or below the
* configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY level (i.e.,
* with the numerical values of priorities equal or higher than
* configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) are allowed to call any
* QP/FreeRTOS services. These ISRs are "kernel-aware".
*
* Conversely, any ISRs prioritized above the
* configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY priority level (i.e., with
* the numerical values of priorities less than
* configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) are never disabled and are
* not aware of the kernel. Such "kernel-unaware" ISRs cannot call any
* QP/FreeRTOS services. The only mechanism by which a "kernel-unaware" ISR
* can communicate with the QF framework is by triggering a "kernel-aware"
* ISR, which can post/publish events.
*
* For more information, see article "Running the RTOS on a ARM Cortex-M Core"
* http://www.freertos.org/RTOS-Cortex-M3-M4.html
*
* NOTE2:
* The User LED is used to visualize the idle loop activity. The brightness
* of the LED is proportional to the frequency of invcations of the idle loop.
* Please note that the LED is toggled with interrupts locked, so no interrupt
* execution time contributes to the brightness of the User LED.
*/

/* BSP_booster_pack interface functions #####################################################*/

/* TODO: bst_pack initialisation


Accelerometer----------------------------
PD2-0 input analogue.
ADC init() //!ADC0 

Microphone-------------------------------
PE5 input analogue.
NOTE: //!ADC0**

joystic----------------------------------
PE4 input analogue
PD3 input analogue
PB5 input analogue 
NOTE: //!ADC0**

Buttons---------------------------------
USER BUTTON 1    PD6 INPUT.
USER BUTTON 2    PD7 INPUT.
JOYSTIC BUTTON 3 PE4 INPUT.

Display---------------------------------
PA4  SPI CS.
PF4  SPI DC. 
PF0  SPI RESET. 


ADC FOR MIC             (J1.6/PE5/AIN8)
ADC FOR JOYSTIC       X (J1.2/PB5/AIN11) and Y (J3.26/PD3/AIN4)
ADC FOR ACCELEROMETER X (J3.23/PD0/AIN7), Y (J3.24/PD1/AIN6), and Z (J3.25/PD2/AIN5)
 */

/* TODO: INPUT/OUTPUT functions
joystic input() 
get inputs from the joystick ADC. can use polling or interrupts


RGB set colour()
set pin pwm
toggle pwm LED (change the pwm value)
toggle digital LED

Buzzer set().

Accelerometer input().
Microphone input().

Display()

write command 8bit
write data 8bit

 */

// ------------BSP_Buzzer_Init------------
// Initialize the GPIO and PWM or timer modules which
// correspond with BoosterPack pin J4.40.  The
// frequency is 2048 Hz, and the duty cycle is
// represented as a 10-bit number.
// Input: duty is 10-bit duty cycle for the buzzer
// Output: none
//TODO: FIX COMMENTS. make it a void void ()
/**Initialise boosterpack buzzer I/O
 * Buzzer is connected to PF2. 
 * Timer1A is used for the buzzer 
 */
static uint32_t PWM_sysclk;  // pwm module clock frequency
void BSP_BP_BuzzerIO_Init(uint16_t duty) {
    if (duty > 1023) {
        return;  // invalid input
    }
    // ***************** pwm1 initialization *****************
    SYSCTL->RCGCGPIO |= 0x0020;  // !activate clock for Port F DONE
    while ((SYSCTL->PRGPIO & 0x20) == 0) {
    };                     // allow time for clock to stabilize
    GPIOF->AFSEL |= 0x04;  // enable alt funct on PF2
    GPIOF->DEN |= 0x04;    // enable digital I/O on PF2
                           // configure PF2 as PWM
    GPIOF->PCTL = (GPIOF->PCTL & 0xFFFFF0FF) + 0x00000500;
    GPIOF->AMSEL &= ~0x04;  // disable analog functionality on PF2

    SYSCTL->RCGCPWM |= 0x02;  // activate clock for pwm
    while ((SYSCTL->PRPWM & 0x02) == 0) {
    };  // allow time for clock to stabilize

    //M1PWM6------------------
    SYSCTL->RCC |= (0x00100000);  /* Enable System Clock Divisor function  */
    SYSCTL->RCC &= ~(0x000E0000); /* clear divide  */
    SYSCTL->RCC |= (0x00020000);  /* divide sysclk by 0002 0000 /4. 6 0000 /4. */
    // update the pwm clock freq divider used.
    PWM_sysclk = (SystemCoreClock / 4);
    PWM1->_3_CTL = 0;           // Disable Generator 0 counter
    PWM1->_3_CTL &= ~(1 << 1);  // select down count mode of counter 1
    // controls when the output is enable and disabled according to Reload and CMPA values
    PWM1->_3_GENA |= (0x000000C0 | 0x00000008); /* Set PWM output when counter matches PWMCMPA clear on reload*/
    PWM1->_3_LOAD = 1;                          // set load value for 50Hz 16MHz/65 = 250kHz and (250KHz/5000)
    //PWM1->_3_CMPA register not configured, pwm output stays at 0.
    PWM1->_3_CTL |= 0x01;      // Enable Generator 3 counter
    PWM1->ENABLE |= (1U << 6); /* Enable PWM1 channel 0 output */
}

// ------------BSP_Buzzer_Set------------
// Set new duty cycle for the buzzer.
// 512 is typical for sound (50% duty cycle).
// 0 is typical for silent (0% duty cycle).
// Input: duty is 10-bit duty cycle for the buzzer
// Output: none
// Assumes: BSP_Buzzer_Init() has been called
//! the duty affects the output volume !!duty ++ = low volume
void BSP_BP_Buzzer_Set(uint16_t duty) {
    if (duty > 65023) {
        return;  // invalid input
    }
    PWM1->ENABLE &= ~(1U << 6);
    // TIMER1->CTL &= ~((0x00000040) | (0x00000001));
    PWM1->_3_LOAD = duty;
    PWM1->_3_CMPA = (duty * 40) / 100;  // defines when output signal is cleared
    PWM1->ENABLE |= (1U << 6);
    //! can use duty as % thus TAMATCHR = duty*pwmcycles
    // TIMER1->CTL |= ((0x00000040) | (0x00000001));
}

/**Buzzer TODO:
 * function to adjust frequency(tone) 
 * assumes no divider is used for frequency 50 mhz.
 */
void BSP_BP_Buzzer_Freq(float32_t freq) {
    // system clock/required frequency.
    float32_t loadValue = (freq);
    uint16_t sysPMW = (loadValue * PWM_sysclk);
    PWM1->_3_LOAD = sysPMW;
    PWM1->_3_CMPA = (sysPMW * 30 / 100);
}
//TODO: Test the frequency on a scope for correctness.
//      Idea. use uint instead of float avoid floating point math?

// pwm on pf2 PWM4 M1PWM6

/**!
 * TODO: led initialisation
 * -- Initialize all leds as digital 
 * -- Initialise all leds as PWM
 * 
 * -- control individual led pwm. 
 */

void BSP_BP_D_LedsInit(void) {
    // initialise  PB3 PC4 PF3
    /* enable Run mode for GPIOB, GPIOC, GPIOF */
    SYSCTL->RCGCGPIO |= PORTB | PORTC | PORTF;
    /* configure BOOSTER PCK Switches and leds */
    /* BSTR_LED_RED_PORT BSTR_LED_RED have been configured above, same PF1 used */
    // PB3
    BSTR_LED_GREEN_PORT->DIR |= (BSTR_LED_GREEN);    /* set as output */
    BSTR_LED_GREEN_PORT->AFSEL &= ~(BSTR_LED_GREEN); /* disable alt funct on PB3 */
    BSTR_LED_GREEN_PORT->PUR &= ~(BSTR_LED_GREEN);   /* disable pull-up on PB3 */
    BSTR_LED_GREEN_PORT->DEN |= (BSTR_LED_GREEN);    /* digital enable */
    // PC4
    BSTR_LED_BLUE_PORT->DIR |= (BSTR_LED_BLUE);    /* set as output */
    BSTR_LED_BLUE_PORT->AFSEL &= ~(BSTR_LED_BLUE); /* disable alt funct on PC4 */
    BSTR_LED_BLUE_PORT->PUR &= ~(BSTR_LED_BLUE);   /* disable pull-up on PC4 */
    BSTR_LED_BLUE_PORT->DEN |= (BSTR_LED_BLUE);    /* digital enable */
    // PF3
    BSTR_LED_RED_PORT->DEN |= (BSTR_LED_RED);    /* digital enable */
    BSTR_LED_RED_PORT->AFSEL &= ~(BSTR_LED_RED); /* disable alt funct on PF3 */
    BSTR_LED_RED_PORT->PUR &= ~(BSTR_LED_RED);   /* disable pull-up on PF3 */
    BSTR_LED_RED_PORT->DIR |= (BSTR_LED_RED);    /* set as output */
    // turn off all leds.
    BSTR_LED_GREEN_PORT->DATA_Bits[BSTR_LED_GREEN] = BSTR_LED_GREEN; /* turn the LED off */
    BSTR_LED_BLUE_PORT->DATA_Bits[BSTR_LED_BLUE] = BSTR_LED_BLUE;    /* turn the LED off */
    BSTR_LED_RED_PORT->DATA_Bits[BSTR_LED_RED] = BSTR_LED_RED;       /* turn the LED off */
}

void BSP_BP_RGB_ledsInit(void) {
    // enable Run mode for GPIOB, GPIOC, GPIOF
    //
    SYSCTL->RCGCGPIO |= PORTB | PORTC | PORTF;

    // configure I/O pins for PWM mode.
    //
    // PB3
    BSTR_LED_GREEN_PORT->DIR |= (BSTR_LED_GREEN);                                         /* set as output */
    BSTR_LED_GREEN_PORT->AFSEL |= (BSTR_LED_GREEN);                                       /* enable alt funct on PB3 */
    BSTR_LED_GREEN_PORT->PUR &= ~(BSTR_LED_GREEN);                                        /* disable pull-up on PB3 */
    BSTR_LED_GREEN_PORT->PCTL |= ((BSTR_LED_GREEN_PORT->PCTL & 0xFFFF0FFF) + 0x00007000); /* configure PB3 as PWM */
    BSTR_LED_GREEN_PORT->AMSEL &= ~(BSTR_LED_GREEN);                                      /* disable analog functionality */
    BSTR_LED_GREEN_PORT->DEN |= (BSTR_LED_GREEN);                                         /* digital enable */
    // PC4
    BSTR_LED_BLUE_PORT->DIR |= (BSTR_LED_BLUE);                                         /* set as output */
    BSTR_LED_BLUE_PORT->AFSEL |= (BSTR_LED_BLUE);                                       /* disable alt funct on PC4 */
    BSTR_LED_BLUE_PORT->PUR &= ~(BSTR_LED_BLUE);                                        /* disable pull-up on PC4 */
    BSTR_LED_BLUE_PORT->PCTL |= ((BSTR_LED_BLUE_PORT->PCTL & 0xFFF0FFFF) + 0x00070000); /* configure PC4 as PWM */
    BSTR_LED_BLUE_PORT->AMSEL &= ~(BSTR_LED_BLUE);                                      /* disable analog functionality */
    BSTR_LED_BLUE_PORT->DEN |= (BSTR_LED_BLUE);                                         /* digital enable */
    // PF3 for Timer1B
    BSTR_LED_RED_PORT->DIR |= (BSTR_LED_RED);                                         /* set as output */
    BSTR_LED_RED_PORT->AFSEL |= (BSTR_LED_RED);                                       /* disable alt funct on PF3 */
    BSTR_LED_RED_PORT->PUR &= ~(BSTR_LED_RED);                                        /* disable pull-up on PF3 */
    BSTR_LED_RED_PORT->PCTL |= ((BSTR_LED_RED_PORT->PCTL & 0xFFFF0FFF) + 0x00007000); /* configure PF3 as PWM */
    BSTR_LED_RED_PORT->AMSEL &= ~(BSTR_LED_RED);                                      /* disable analog functionality */
    BSTR_LED_RED_PORT->DEN |= (BSTR_LED_RED);                                         /* digital enable */

    // enable Timers for pwm clock
    //
    // ***************** Timer1B (PF3) initialization *****************
    SYSCTL->RCGCTIMER |= 0x02;  // activate clock for Timer1

    while ((SYSCTL->PRTIMER & 0x02) == 0) {
    };                             // allow time for clock to stabilize
    TIMER1->CTL &= ~(0x00000100);  // disable Timer1B during setup
    TIMER1->CFG = 0x00000004;      // configure for 16-bit timer mode
                                   // configure for alternate (PWM) mode
    TIMER1->TBMR = (0x00000008 | 0x00000002);
    TIMER1->TBILR = PWM_CYCLES;  // defines when output signal is set
    //TIMER1->TBMATCHR register is not set, keeps pwm output at 0
    //
    // enable Timer1B 16-b, PWM, inverted to match comments
    TIMER1->CTL |= (0x00004000 | 0x00000100);
    //==============================================================================================

    // ***************** Timer3B (PB3) initialization *****************
    SYSCTL->RCGCTIMER |= 0x08;  // activate clock for Timer1

    while ((SYSCTL->PRTIMER & 0x08) == 0) {
    };                             // allow time for clock to stabilize
    TIMER3->CTL &= ~(0x00000100);  // disable Timer3B during setup
    TIMER3->CFG = 0x00000004;      // configure for 16-bit timer mode
                                   // configure for alternate (PWM) mode
    TIMER3->TBMR = (0x00000008 | 0x00000002);
    TIMER3->TBILR = PWM_CYCLES;  // defines when output signal is set
    //TIMER3->TBMATCHR register is not set, keeps pwm output at 0
    //
    // enable Timer1B 16-b, PWM, inverted to match comments
    TIMER3->CTL |= (0x00004000 | 0x00000100);  // enable PWM

    //==============================================================================================

    // ***************** Wide Timer0A (PC4) initialization *****************
    SYSCTL->RCGCWTIMER |= 0x01;  // activate clock for Wide Timer0

    while ((SYSCTL->PRWTIMER & 0x01) == 0) {
    };                             // allow time for clock to stabilize
    WTIMER0->CTL &= ~(0x0000001);  // disable Timer3B during setup
    WTIMER0->CFG = 0x00000004;     // configure for 32-bit timer mode
                                   // configure for alternate (PWM) mode
    WTIMER0->TAMR = (0x00000008 | 0x00000002);
    WTIMER0->TAILR = PWM_CYCLES;  // defines when output signal is set
    //WTIMER0->TAMATCHR register is not set, keeps pwm output at 0
    //
    // enable Wide Timer0A 32-b, PWM, inverted to match comments
    WTIMER0->CTL |= (0x00000040 | 0x00000001);
    //==============================================================================================

    // End of function ====================================================================//
}

// set pwm for each led
// assumes that led rgb init has been called
// takes duty cycle as percentage.
// ! check! duty == 0 pwm on for 20ns the lowest to disable use pwm disable.
void BSP_BP_LedRedDuty(uint8_t dutyCycle) {
    if (dutyCycle > 100) {
        /* duty cycle is out of range do nothing */
        return;  // todo: can assert here.
    }
    TIMER1->TBMATCHR = ((PWM_CYCLES * dutyCycle) / 100) - 1;
}
void BSP_BP_LedGreenDuty(uint8_t dutyCycle) {
    if (dutyCycle > 100) {
        /* duty cycle is out of range do nothing */
        return;  // todo: can assert here.
    }
    TIMER3->TBMATCHR = ((PWM_CYCLES * dutyCycle) / 100) - 1;
}
void BSP_BP_LedBlueDuty(uint8_t dutyCycle) {
    if (dutyCycle > 100) {
        /* duty cycle is out of range do nothing */
        return;  // todo: can assert here.
    }
    WTIMER0->TAMATCHR = ((PWM_CYCLES * dutyCycle) / 100) - 1;
}

/**
 * TODO: ERROR HANDLING for pwm usage
 * before setting duty cycle check if pwm timer is enabled first
 * we can assert if timer is not enable or just enble then set new duty
 *  = time enable and disable() for power mangment when led duty is set to 0.
 *  = this will reduce the power consumed by the timer.
 */

// Booster pack display using spi============================================

// The Data/Command pin must be valid when the eighth bit is
// sent.  The SSI module has hardware input and output FIFOs
// that are 8 locations deep; however, they are not used in
// this implementation.  Each function first stalls while
// waiting for any pending SSI2 transfers to complete.  Once
// the SSI2 module is idle, it then prepares the Chip Select
// pin for the LCD and the Data/Command pin.  Next it starts
// transmitting the data or command.  Finally once the
// hardware is idle again, it sets the chip select pin high
// as required by the serial protocol.  This is a
// significant change from previous implementations of this
// function.  It is less efficient without the FIFOs, but it
// should ensure that the Chip Select and Data/Command pin
// statuses all match the byte that is actually being
// transmitted.
// NOTE: These functions will crash or stall indefinitely if
// the SSI2 module is not initialized and enabled.

// This is a helper function that sends an 8-bit command to the LCD.
// Inputs: c  8-bit code to transmit
// Outputs: 8-bit reply
// Assumes: SSI2 and ports have already been initialized and enabled
uint8_t BSP_BP_SPIwritecommand(uint8_t c) {
    // wait until SSI2 not busy/transmit FIFO empty
    // todo remove while loop, blocking
    while ((SSI2->SR & 0x00000010) == 0x00000010) {
    };
    BSP_BP_TFT_CS_LOW();
    BSP_BP_TFT_DC_COMMAND();
    SSI2->DR = c;  // data out
    while ((SSI2->SR & 0x00000004) == 0) {
    };  // wait until response
    BSP_BP_TFT_CS_HIGH();
    return (uint8_t)SSI2->DR;  // return the response
}
// This is a helper function that sends a piece of 8-bit data to the LCD.
// Inputs: c  8-bit data to transmit
// Outputs: 8-bit reply
// Assumes: SSI2 and ports have already been initialized and enabled
uint8_t BSP_BP_SPIwritedata(uint8_t c) {
    // wait until SSI2 not busy/transmit FIFO empty
    // todo remove while loop.
    while ((SSI2->SR & 0x00000010) == 0x00000010) {
    };
    BSP_BP_TFT_CS_LOW();
    BSP_BP_TFT_DC_DATA();
    SSI2->DR = c;  // data out
    while ((SSI2->SR & 0x00000004) == 0) {
    };  // wait until response
    BSP_BP_TFT_CS_HIGH();
    return (uint8_t)SSI2->DR;  // return the response
}

//! delay function required for initialisation of LCD display.
//BUG: not sure if this will work with the arm none eabi gcc compiler!
// try the TI compiler version

// delay function from sysctl.c
// which delays 3.3*ulCount cycles
// ulCount=23746 => 1ms = 23746*3.3cycle/loop/80,000

//Code Composer Studio Code
// void parrotdelay(uint32_t ulCount) {
//     __asm(
//         "    subs    r0, #1\n"
//         "    bne     parrotdelay\n"
//         "    bx      lr\n");
// }
// delay function from sysctl.c
// which delays 3.3*ulCount cycles
// ulCount=23746 => 1ms = 23746*3.3cycle/loop/80,000
#ifdef __TI_COMPILER_VERSION__
//Code Composer Studio Code
void parrotdelay(uint32_t ulCount) {
    __asm(
        "    subs    r0, #1\n"
        "    bne     Delay\n"
        "    bx      lr\n");
}

#else
//Keil uVision Code
__asm void
parrotdelay(uint32_t ulCount) {
    subs r0, #1 bne parrotdelay
                 bx lr
}

#endif
// ------------BSP_Delay1ms------------
// Simple delay function which delays about n
// milliseconds.
// Inputs: n  number of 1 msec to wait
// Outputs: none
// Note: this is based on a 80Mhz sysClk
void BSP_Delay1ms(uint32_t n) {
    while (n) {
        parrotdelay(23746);  // 1 msec, tuned at 80 MHz, originally part of LCD module
        n--;
    }
}

// Initialization SPI interface for BP TFT display.
void BSP_BP_SPI_TFT_Init(void) {
    // toggle RST low to reset; CS low so it'll listen to us
    // SSI2Fss is not available, so use GPIO on PA4
    SYSCTL->RCGCGPIO |= 0x00000023;  // 1) activate clock for Ports F, B, and A
    while ((SYSCTL->PRGPIO & 0x23) != 0x23) {
    };                                   // allow time for clocks to stabilize
    BP_LCD_RST_PORT->LOCK = 0x4C4F434B;  // 2a) unlock GPIO Port F
    BP_LCD_RST_PORT->CR = 0x1F;          // allow changes to PF4-0
                                         // 2b) no need to unlock PF4, PB7, PB4, or PA4
    BP_LCD_RST_PORT->AMSEL &= ~0x11;     // 3a) disable analog on PF4,0
    BP_SPI_DATA_PORT->AMSEL &= ~0x90;    // 3b) disable analog on PB7,4
    BP_SPI_CS_PORT->AMSEL &= ~0x10;      // 3c) disable analog on PA4
                                         // 4a) configure PF4,0 as GPIO
    BP_LCD_RST_PORT->PCTL = (BP_LCD_RST_PORT->PCTL & 0xFFF0FFF0) + 0x00000000;
    // 4b) configure PB7,4 as SSI
    BP_SPI_DATA_PORT->PCTL = (BP_SPI_DATA_PORT->PCTL & 0x0FF0FFFF) + 0x20020000;
    // 4c) configure PA4 as GPIO
    BP_SPI_CS_PORT->PCTL = (BP_SPI_CS_PORT->PCTL & 0xFFF0FFFF) + 0x00000000;
    BP_LCD_RST_PORT->DIR |= 0x11;     // 5a) make PF4,0 output
    BP_SPI_CS_PORT->DIR |= 0x10;      // 5b) make PA4 output
    BP_LCD_RST_PORT->AFSEL &= ~0x11;  // 6a) disable alt funct on PF4,0
    BP_SPI_DATA_PORT->AFSEL |= 0x90;  // 6b) enable alt funct on PB7,4
    BP_SPI_CS_PORT->AFSEL &= ~0x10;   // 6c) disable alt funct on PA4
    BP_LCD_RST_PORT->DEN |= 0x11;     // 7a) enable digital I/O on PF4,0
    BP_SPI_DATA_PORT->DEN |= 0x90;    // 7b) enable digital I/O on PB7,4
    BP_SPI_CS_PORT->DEN |= 0x10;      // 7c) enable digital I/O on PA4

    // reset display
    BSP_BP_TFT_CS_LOW();
    BSP_BP_TFT_RESET_HIGH();
    BSP_Delay1ms(50);
    BSP_BP_TFT_RESET_LOW();
    BSP_Delay1ms(50);
    BSP_BP_TFT_RESET_HIGH();
    BSP_Delay1ms(50);
    BSP_BP_TFT_CS_HIGH();

    // activate clock for SSI2
    SYSCTL->RCGCSSI |= 0x00000004;
    // allow time for clock to stabilize
    while ((SYSCTL->PRSSI & 0x00000004) == 0) {
    };
    SSI2->CR1 &= ~0x00000002;  // disable SSI
    SSI2->CR1 &= ~0x00000004;  // master mode
                               // configure for clock from source PIOSC for baud clock source
    SSI2->CC = (SSI2->CC & ~0x0000000F) + 0x00000005;
    // clock divider for 4 MHz SSIClk (16 MHz PIOSC/4)
    // PIOSC/(CPSDVSR*(1+SCR))
    // 16/(4*(1+0)) = 4 MHz
    SSI2->CPSR = (SSI2->CPSR & ~0x000000FF) + 4;  // must be even number
    SSI2->CR0 &= ~(0x0000FF00 |                   // SCR = 0 (4 Mbps data rate)
                   0x00000080 |                   // SPH = 0
                   0x00000040);                   // SPO = 0
                                                  // FRF = Freescale format
    SSI2->CR0 = (SSI2->CR0 & ~0x00000030) + 0x00000000;
    // DSS = 8-bit data
    SSI2->CR0 = (SSI2->CR0 & ~0x0000000F) + 0x00000007;
    SSI2->CR1 |= 0x00000002;  // enable SSI
}

// END LCD display functions===================================================//
//

// ADC0 initialisation==========================================================//
// There are six analog inputs on the Educational BoosterPack MKII:
// microphone (J1.6/PE5/AIN8)
// joystick X (J1.2/PB5/AIN11) and Y (J3.26/PD3/AIN4)
// accelerometer X (J3.23/PD0/AIN7), Y (J3.24/PD1/AIN6), and Z (J3.25/PD2/AIN5)
// All six initialization functions can use this general ADC
// initialization.  The joystick uses sample sequencer 1,
// the accelerometer sample sequencer 2, and the microphone
// uses sample sequencer 3.
static void adcinit0(void) {
    SYSCTL->RCGCADC |= 0x00000001;  // 1) activate ADC0
    while ((SYSCTL->PRADC & 0x01) == 0) {
    };                     // 2) allow time for clock to stabilize
                           // 3-7) GPIO initialization in more specific functions
    ADC0->PC &= ~0xF;      // 8) clear max sample rate field
    ADC0->PC |= 0x2;       //    configure for 125K samples/sec ** see data sheet
    ADC0->SSPRI = 0x3210;  // 9) Sequencer 3 is lowest priority
                           // 10-15) sample sequencer initialization in more specific functions
}

// BSP Booster pack Joystick functions======================================
//
void BSP_BP_Joystick_Init(void) {
    /* enable Run mode for GPIOE, GPIOB, GPIOD */
    SYSCTL->RCGCGPIO |= PORTE | PORTB | PORTD;
    /* booster pack User Switches initialization */

    BSP_BP_JST_BTN_PORT->DIR &= ~(BSP_BP_JST_BTN_PIN); /*  set direction: input */

    ROM_GPIOPadConfigSet(GPIOE_BASE, (BSP_BP_JST_BTN_PIN),
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    BSP_BP_ADC_Y_PORT->AMSEL |= 0x08;  // 3b) enable analog on PD3
    BSP_BP_ADC_X_PORT->AMSEL |= 0x20;  // 3c) enable analog on PB5

    BSP_BP_ADC_Y_PORT->DIR &= ~0x08;  // 5b) make PD3 input
    BSP_BP_ADC_X_PORT->DIR &= ~0x20;  // 5c) make PB5 input

    BSP_BP_ADC_Y_PORT->AFSEL |= 0x08;  // 6b) enable alt funct on PD3
    BSP_BP_ADC_X_PORT->AFSEL |= 0x20;  // 6c) enable alt funct on PB5
    BSP_BP_ADC_Y_PORT->DEN &= ~0x08;   // 7b) enable analog functionality on PD3
    BSP_BP_ADC_X_PORT->DEN &= ~0x20;   // 7c) enable analog functionality on PB5

    adcinit0();  // 8-9) general ADC initialization

    ADC0->ACTSS &= ~0x0002;  // 10) disable sample sequencer 1
    ADC0->EMUX &= ~0x00F0;   // 11) seq1 is software trigger
    ADC0->SSMUX1 = 0x004B;   // 12) set channels for SS1
    ADC0->SSCTL1 = 0x0060;   // 13) no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
    ADC0->IM &= ~0x0002;     // 14) disable SS1 interrupts
    ADC0->ACTSS |= 0x0002;   // 15) enable sample sequencer 1
}

// ------------BSP_Joystick_Input------------
// Read and return the immediate status of the
// joystick.  Button de-bouncing for the Select
// button is not considered.  The joystick X- and
// Y-positions are returned as 10-bit numbers,
// even if the ADC on the LaunchPad is more precise.
// Input: x is pointer to store X-position (0 to 1023)
//        y is pointer to store Y-position (0 to 1023)
//        select is pointer to store Select status (0 if pressed)
// Output: none
// Assumes: BSP_Joystick_Init() has been called
// TODO: clean up the blocking while loop.
void BSP_Joystick_Input(uint16_t *x, uint16_t *y) {
    // ADC0->PSSI = 0x0002;  // 1) initiate SS1
    // while ((ADC0->RIS & 0x02) == 0) {
    // };                         // 2) wait for conversion done

    // static uint32_t adcLPS = 0; // low pass filtered ADC reading
    // static uint32_t joystickX = 0; // last joystick x position
    // static uint32_t joystickY = 0; // last joystick y position
    // unsigned long tmp;
    /* 1st order low-pass filter: time constant ~= 2^n samples
        * TF = (1/2^n)/(z-((2^n - 1)/2^n)),
        * e.g., n=3, y(k+1) = y(k) - y(k)/8 + x(k)/8 => y += (x - y)/8
        */

    *x = ADC0->SSFIFO1 >> 2;  // read first result. (>>2) divide by 4

    *y = ADC0->SSFIFO1 >> 2;  // 3b) read second result
    ADC0->ISC = 0x0002;       // 4) acknowledge completion
}

// trigger joystick ADC
void BSP_Joystick_Trigger(void) {
    ADC0->PSSI |= (1 << 1); /* Enable SS2 conversion or start sampling data from AN0 */
}

// ADC initialization for onboard temp sensor //! used for mic.
// Using intrrupt mode.
// static void mcuTempSensorInit(void) {
//     SYSCTL->RCGCADC |= 1; /* enable clock to ADC0 */
//     /* initialize ADC0 */
//     ADC0->ACTSS &= ~8;         /* disable SS3 during configuration */
//     ADC0->EMUX &= ~0xF000;     /* software trigger conversion seq 0 */
//     ADC0->SSMUX3 = 0x0008;     /* get input from channel 0 */
//     ADC0->SSCTL3 |= 0x0E;      /* take chip temperature, set flag at 1st sample */
//     ADC0->IM |= 0x0008;        //  enable SS3 interrupts
//     ADC0->ACTSS |= (1U << 3);  //  enable sample sequencer 3
// }

// trigger the ADC to sample the MCU temperature sensor.
void BSP_SystemTempGet(void) {
    ADC0->PSSI |= (1 << 3); /* Enable SS3 conversion or start sampling data from AN0 */
}

// ------------BSP_Accelerometer_Init------------
// Initialize three ADC pins, which correspond with
// 4x hardware oversampling
// BoosterPack pins J3.23 (X) (PD0/AIN7), J3.24 (Y) (PD1/AIN6), and
// J3.25 (Z) (PD2/AIN5).
// Input: none
// Output: none
void BSP_Accelerometer_Init(void) {
    adcinit0();
    SYSCTL->RCGCGPIO |= 0x00000008;  // 1) activate clock for Port D
    while ((SYSCTL->PRGPIO & 0x08) == 0) {
    };                       // allow time for clock to stabilize
                             // 2) no need to unlock PD2-0
    GPIOD->AMSEL |= 0x07;    // 3) enable analog on PD2-0
                             // 4) configure PD2-0 as ?? (skip this line because PCTL is for digital only)
    GPIOD->DIR &= ~0x07;     // 5) make PD2-0 input
    GPIOD->AFSEL |= 0x07;    // 6) enable alt funct on PD2-0
    GPIOD->DEN &= ~0x07;     // 7) enable analog functionality on PD2-0
    adcinit0();              // 8-9) general ADC initialization
    ADC0->ACTSS &= ~0x0004;  // 10) disable sample sequencer 2
    ADC0->EMUX &= ~0x0F00;   // 11) seq2 is software trigger
    ADC0->SSMUX2 = 0x0567;   // 12) set channels for SS2
    ADC0->SAC = 0x03;        // 13) Sample Averaging 0x02==4x hardware oversampling
    ADC0->SSCTL2 = 0x0600;   // 14) no D0 END0 IE0 TS0 D1 END1 IE1 TS1 D2 TS2, yes IE2 END2
    ADC0->IM &= ~0x0004;     // 15) disable SS2 interrupts
    ADC0->ACTSS |= 0x0004;   // 16) enable sample sequencer 2
}

// ------------BSP_Accelerometer_Input------------
// Read and return the immediate status of the
// accelerometer.  The accelerometer X-, Y-, and
// Z-measurements are returned as 10-bit numbers,
// even if the ADC on the LaunchPad is more precise.
// Input: x is pointer to store X-measurement (0 to 1023)
//        y is pointer to store Y-measurement (0 to 1023)
//        z is pointer to store Z-measurement (0 to 1023)
// Output: none
// Assumes: BSP_Accelerometer_Init() has been called
// TODO: remove blocking while loop.
void BSP_Accelerometer_Input(uint16_t *x, uint16_t *y, uint16_t *z) {
    // ADC0->PSSI = 0x0004;  // 1) initiate SS2
    // // TODO: add while loop timeout
    // while ((ADC0->RIS & 0x04) == 0) {};// 2) wait for conversion done
    //! SSFIFO2 >> 2 as result is a 10-bit value and fifo data is 12-bits
    *x = ADC0->SSFIFO2 >> 2;  // 3a) read first result
    *y = ADC0->SSFIFO2 >> 2;  // 3b) read second result
    *z = ADC0->SSFIFO2 >> 2;  // 3c) read third result
    ADC0->ISC = 0x0004;       // 4) acknowledge completion
}

// trigger the ADC to sample the Accelerometer sensor.
void BSP_AccelerometerGet(void) {
    ADC0->PSSI |= (1 << 2);  // initiate SS2
}

// TODO: isr for accelerometer ADC

// TODO: use the ADC1 for the microphone. can set higher sample rate. change ADC0 to ADC1.
//       isr for microphone

// ADC1 initialisation==========================================================//
// analog input for microphone (J1.6/PE5/AIN8) on the Educational BoosterPack MKII:
// microphone uses sample sequencer 3.
static void l_adcinit1(void) {
    SYSCTL->RCGCADC |= 0x00000002;  // 1) activate ADC1
    while ((SYSCTL->PRADC & 0x02) == 0) {
    };                     // 2) allow time for clock to stabilize
                           // 3-7) GPIO initialization in more specific functions
    ADC1->PC &= ~0xF;      // 8) clear max sample rate field
    ADC1->PC |= 0x3;       //    configure for 500K samples/sec ** see data sheet
                           // ADC1->PP |= 0x05;      // Maximum ADC Sample Rate 0x05 500 ksps
    ADC1->SSPRI = 0x3210;  // 9) Sequencer 3 is lowest priority
                           // 10-15) sample sequencer initialization in more specific functions
}

// ------------BSP_Microphone_Init------------
// Initialize one ADC pin, which corresponds with
// BoosterPack pin J1.6. tm4c123 pin PE5.
// Input: none
// Output: none
void BSP_Microphone_Init(void) {
    l_adcinit1();
    SYSCTL->RCGCGPIO |= 0x00000010;  // 1) activate clock for Port E
    while ((SYSCTL->PRGPIO & 0x10) == 0) {
    };                       // allow time for clock to stabilize
                             // 2) no need to unlock PE5
    GPIOE->AMSEL |= 0x20;    // 3) enable analog on PE5
                             // 4) configure PE5 as ?? (skip this line because PCTL is for digital only)
    GPIOE->DIR &= ~0x20;     // 5) make PE5 input
    GPIOE->AFSEL |= 0x20;    // 6) enable alt funct on PE5
    GPIOE->DEN &= ~0x20;     // 7) enable analog functionality on PE5
    l_adcinit1();            // 8-9) general ADC initialization
    ADC1->ACTSS &= ~0x0008;  // 10) disable sample sequencer 3
    ADC1->EMUX &= ~0xF000;   // 11) clear seq3 trigger
    ADC1->EMUX |= 0x6000;   // 11) seq3 is PWM gen 0 triggered
    ADC1->SSMUX3 = 0x0008;   // 12) set channels for SS3
    ADC1->SSCTL3 = 0x0006;   // 13) no D0 TS0, yes IE0 END0
    ADC1->TSSEL |= 0x6000;    // 14) PWM gen 3 Trigger Source
    //ADC1->SAC |= 0x6;        // 14) 4x hardware oversampling
    //ADC1->PP |= 0x00300000;  // 14) Resolution 0x30 0000==12-bit
    ADC1->IM |= 0x0008;     // 15) enable SS3 interrupts
    ADC1->ACTSS |= 0x0008;  // 16) enable sample sequencer 3
// initialize pwm0 for triggering
    BSP_ADC_PWM_Init();
}

// TODO: documentation as a polling/blocking call
// ------------BSP_Microphone_Input------------
// Read and return the immediate status of the
// microphone.  The sound measurement is returned
// as a 10-bit number, even if the ADC on the
// LaunchPad is more precise.
// Input: mic is pointer to store sound measurement (0 to 1023)
// Output: none
// Assumes: BSP_Microphone_Init() has been called
void BSP_Microphone_Input(uint16_t *mic) {
    // ADC0->PSSI = 0x0008;  // 1) initiate SS3
    // while ((ADC0->RIS & 0x08) == 0) {
    // };                           // 2) wait for conversion done
    *mic = ADC1->SSFIFO3 >> 2;  // 3) read result
    ADC1->ISC = 0x0008;         // 4) acknowledge completion
}

// trigger the ADC to sample the mic input.
void BSP_Microphone_Get(void) {
    ADC1->PSSI = 0x0008;  // 1) initiate SS3
}

// I2C initialization==============================================

// There are two I2C devices on the Educational BoosterPack MKII:
// OPT3001 Light Sensor
// !TMP006 Temperature sensor not populated.
// Both initialization functions can use this general I2C
// initialization.
#define MAXRETRIES 5  // number of receive attempts before giving up
void static i2cinit(void) {
    SYSCTL->RCGCI2C |= 0x0002;              // 1a) activate clock for I2C1
    SYSCTL->RCGCGPIO |= PORTA;              // 1b) activate clock for Port A
    while ((SYSCTL->PRGPIO & 0x01) == 0) {  // TODO: add while loop timeout
    };                                      // allow time for clock to stabilize
                                            // 2) no need to unlock PA7-6
    GPIOA->AMSEL &= ~0xC0;                  // 3) disable analog functionality on PA7-6
                                            // 4) configure PA7-6 as I2C1
    GPIOA->PCTL = (GPIOA->PCTL & 0x00FFFFFF) + 0x33000000;
    GPIOA->ODR |= 0x80;      // 5) enable open drain on PA7 only
    GPIOA->AFSEL |= 0xC0;    // 6) enable alt funct on PA7-6
    GPIOA->DEN |= 0xC0;      // 7) enable digital I/O on PA7-6
    I2C1->MCR = 0x00000010;  // 8) master function enable
    I2C1->MTPR = 39;         // 9) configure for 100 kbps clock
                             // 20*(TPR+1)*12.5ns = 10us, with TPR=39
}

static void L_waitTillReady(void) {
    while (I2C1->MCS & I2C_MCS_BUSY) {  // TODO: add while loop timeout
    };
}
// receives two bytes from specified slave
// Note for HMC6352 compass only:
// Used with 'A' commands
// Note for TMP102 thermometer only:
// Used to read the contents of the pointer register
uint16_t BSP_I2C_Recv2(int8_t slave) {
    uint8_t data1, data2;
    int retryCounter = 1;
    do {
        L_waitTillReady();                // wait for I2C ready
        I2C1->MSA = (slave << 1) & 0xFE;  // MSA[7:1] is slave address
        I2C1->MSA |= 0x01;                // MSA[0] is 1 for receive

        I2C1->MCS = (I2C_MCS_ACK      // positive data ack
                                      //& ~I2C_MCS_STOP    // no stop
                     | I2C_MCS_START  // generate start/restart
                     | I2C_MCS_RUN);  // master enable
        L_waitTillReady();            // wait for transmission done
        data1 = (I2C1->MDR & 0xFF);   // MSB data sent first

        I2C1->MCS = (I2C_MCS_STOP  // generate stop
                     //& ~I2C_MCS_ACK     // negative data ack (last byte)
                     //& ~I2C_MCS_START   // no start/restart
                     | I2C_MCS_RUN);      // master enable
        L_waitTillReady();                // wait for transmission done
        data2 = (I2C1->MDR & 0xFF);       // LSB data sent last
        retryCounter = retryCounter + 1;  // increment retry counter
    }                                     // repeat if error
    while (((I2C1->MCS & (I2C_MCS_ADRACK | I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
    return (data1 << 8) + data2;  // usually returns 0xFFFF on error
}

// sends one byte to specified slave
// Note for HMC6352 compass only:
// Used with 'S', 'W', 'O', 'C', 'E', 'L', and 'A' commands
//  For 'A' commands, I2C_Recv2() should also be called
// Note for TMP102 thermometer only:
// Used to change the pointer register
// Returns 0 if successful, nonzero if error
uint16_t BSP_I2C_Send1(int8_t slave, uint8_t data1) {
    while (I2C1->MCS & I2C_MCS_BUSY) {
    };                                // wait for I2C ready
    I2C1->MSA = (slave << 1) & 0xFE;  // MSA[7:1] is slave address
    I2C1->MSA &= ~0x01;               // MSA[0] is 0 for send
    I2C1->MDR = data1 & 0xFF;         // prepare first byte
    I2C1->MCS = (0
                 //                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                 | I2C_MCS_STOP         // generate stop
                 | I2C_MCS_START        // generate start/restart
                 | I2C_MCS_RUN);        // master enable
    while (I2C1->MCS & I2C_MCS_BUSY) {  // TODO: add while loop timeout
    };                                  // wait for transmission done
        // return error bits
    return (I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR));
}

uint16_t BSP_I2C_Send3(int8_t slave, uint8_t data1, uint8_t data2, uint8_t data3) {
    L_waitTillReady();
    I2C1->MSA = (slave << 1) & 0xFE;  // MSA[7:1] is slave address
    I2C1->MSA &= ~0x01;               // MSA[0] is 0 for send
    I2C1->MDR = data1 & 0xFF;         // prepare first byte

    I2C1->MCS = (I2C_MCS_START    // generate start/restart
                 | I2C_MCS_RUN);  // master enable
                                  // & ~I2C_MCS_ACK    // no data ack (no data on send)
                                  // & ~I2C_MCS_STOP   // no stop
    L_waitTillReady();            // wait for transmission done
        // check error bits
    if ((I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR)) != 0) {
        // send stop if nonzero
        I2C1->MCS = (I2C_MCS_STOP);  // stop
                                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                                     //  & ~I2C_MCS_START   // no start/restart
                                     //  & ~I2C_MCS_RUN     // master disable

        // return error bits if nonzero
        return (I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR));
    }
    I2C1->MDR = data2 & 0xFF;  // prepare second byte

    I2C1->MCS = (I2C_MCS_RUN);  // master enable

    //& ~I2C_MCS_ACK     // no data ack (no data on send)
    //& ~I2C_MCS_STOP    // no stop
    //& ~I2C_MCS_START   // no start/restart
    L_waitTillReady();  // wait for transmission done
        // check error bits
    if ((I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR)) != 0) {
        // send stop if nonzero
        I2C1->MCS = (0 | I2C_MCS_STOP);  // stop

        // & ~I2C_MCS_ACK    // no data ack (no data on send)
        // & ~I2C_MCS_START  // no start/restart
        // & ~I2C_MCS_RUN   // master disable

        // return error bits if nonzero
        return (I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR));
    }
    I2C1->MDR = data3 & 0xFF;  // prepare third byte

    I2C1->MCS = (I2C_MCS_STOP     // generate stop
                 | I2C_MCS_RUN);  // master enable
                                  // & ~I2C_MCS_ACK     // no data ack (no data on send)
                                  //& ~I2C_MCS_START   // no start/restart
    L_waitTillReady();            // wait for transmission done
        // return error bits
    return (I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR));
}

// ------------BSP_LightSensor_Init------------
// Initialize a GPIO pin for input, which corresponds
// with BoosterPack pins J1.8 (Light Sensor interrupt).
// Initialize two I2C pins, which correspond with
// BoosterPack pins J1.9 (SCL) and J1.10 (SDA).
// Input: none
// Output: none
// TODO: make interrupt pin/ configure ISR and NVIC
// TODO: use #defined IO definetions (port.h).
void BSP_LightSensor_Init(void) {  // bsp light sensor IO inti
    i2cinit();
    // 1) activate clock for Port A (done in i2cinit())
    // allow time for clock to stabilize (done in i2cinit())
    // 2) no need to unlock PA5
    GPIOA->AMSEL &= ~0x20;  // 3) disable analog on PA5
                            // 4) configure PA5 as GPIO
    GPIOA->PCTL = (GPIOA->PCTL & 0xFF0FFFFF) + 0x00000000;
    GPIOA->DIR &= ~0x20;    // 5) make PA5 input
    GPIOA->AFSEL &= ~0x20;  // 6) disable alt funct on PA5
    GPIOA->DEN |= 0x20;     // 7) enable digital I/O on PA5
}

// light sensor data DATA_READY
int BSP_LightSensor_INTstatus(void) {
    return (BSP_BP_OPT3001_IRQ_PORT->DATA_Bits[BSP_BP_OPT3001_IRQ_PIN] == BSP_BP_OPT3001_IRQ_PIN) ? 1U : 0U;
}

// UART1 Initialisation ======================================================================
//

#define FIFOSIZE 256   // size of the FIFOs (must be power of 2)
#define FIFOSUCCESS 1  // return value on success
#define FIFOFAIL 0     // return value on failure
uint32_t RxPutI;       // should be 0 to SIZE-1
uint32_t RxGetI;       // should be 0 to SIZE-1
uint32_t RxFifoLost;   // should be 0
uint8_t RxFIFO[FIFOSIZE];
void RxFifo_Init(void) {
    RxPutI = RxGetI = 0;  // empty
    RxFifoLost = 0;       // occurs on overflow
}
int RxFifo_Put(uint8_t data) {
    if (((RxPutI + 1) & (FIFOSIZE - 1)) == RxGetI) {
        RxFifoLost++;
        return FIFOFAIL;  // fail if full
    }
    RxFIFO[RxPutI] = data;                   // save in FIFO
    RxPutI = (RxPutI + 1) & (FIFOSIZE - 1);  // next place to put
    return FIFOSUCCESS;
}
int RxFifo_Get(uint8_t *datapt) {
    if (RxPutI == RxGetI) return 0;          // fail if empty
    *datapt = RxFIFO[RxGetI];                // retrieve data
    RxGetI = (RxGetI + 1) & (FIFOSIZE - 1);  // next place to get
    return FIFOSUCCESS;
}

//------------UART1_InStatus------------
// Returns how much data available for reading
// Input: none
// Output: number of elements in receive FIFO
uint32_t UART1_InStatus(void) {
    return ((RxPutI - RxGetI) & (FIFOSIZE - 1));
}

//TODO: try 16bit uart, also change uart length enum in UartEvt struct for this test.
//TODO: Use 115200 buad rate.
//------------UART1_Init------------
// Initialize the UART1 for 115,200 baud rate (assuming 50 MHz clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART1_Init(void) {
    SYSCTL->RCGCUART |= 0x02;        // activate UART1
    SYSCTL->RCGCGPIO |= 0x02;        // activate port B
    RxFifo_Init();                   // initialize empty FIFO
    UART1->CTL &= ~UART_CTL_UARTEN;  // disable UART
    UART1->IBRD = 325;               // IBRD = int(50,000,000 / (16 * 115200)) = int(27.126736) (interger part)
    UART1->FBRD = 33;                // FBRD = round(0.126736 * 64) = 26 (fractional part)
                                     // 8 bit word length (no parity bits, one stop bit, FIFOs)
    UART1->LCRH = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    UART1->IFLS &= ~0x3F;  // clear TX and RX interrupt FIFO level fields
                           // configure interrupt for TX FIFO <= 1/8 full
                           // configure interrupt for RX FIFO >= 1/8 full
    UART1->IFLS += (UART_IFLS_TX1_8 | UART_IFLS_RX1_8);
    // enable RX FIFO interrupts and RX time-out interrupt
    UART1->IM |= (UART_IM_RXIM | UART_IM_RTIM);
    UART1->CTL |= 0x301;   // enable UART
    GPIOB->AFSEL |= 0x03;  // enable alt funct on PB1-0
    GPIOB->DEN |= 0x03;    // enable digital I/O on PB1-0
                           // configure PB1-0 as UART
    GPIOB->PCTL = (GPIOB->PCTL & 0xFFFFFF00) + 0x00000011;
    GPIOB->AMSEL &= ~0x03;  // disable analog functionality on PB
}
// copy from hardware RX FIFO to software RX FIFO
// stop when hardware RX FIFO is empty or software RX FIFO is full
void static copyHardwareToSoftware(void) {
    uint8_t letter;
    while (((UART1->FR & UART_FR_RXFE) == 0) && (UART1_InStatus() < (FIFOSIZE - 1))) {
        letter = UART1->DR;
        RxFifo_Put(letter);
    }
}

// input ASCII character from UART
// spin if RxFifo is empty
uint8_t UART1_InChar(void) {
    uint8_t letter;
    while (RxFifo_Get(&letter) == FIFOFAIL) {
    };
    return (letter);
}
//------------UART1_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART1_OutChar(uint8_t data) {
    while ((UART1->FR & UART_FR_TXFF) != 0)
        ;
    UART1->DR = data;
}

//------------UART1_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART1_OutString(uint8_t *pt) {
    while (*pt) {
        UART1_OutChar(*pt);
        pt++;
    }
}

//-----------------------UART1_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART1_OutUDec(uint32_t n) {
    // This function uses recursion to convert decimal number
    //   of unspecified length as an ASCII string
    if (n >= 10) {
        UART1_OutUDec(n / 10);
        n = n % 10;
    }
    UART1_OutChar(n + '0'); /* n is between 0 and 9 */
}

//------------UART1_FinishOutput------------
// Wait for all transmission to finish
// Input: none
// Output: none
void UART1_FinishOutput(void) {
    // Wait for entire tx message to be sent
    // UART Transmit FIFO Empty =1, when Tx done
    while ((UART1->FR & UART_FR_TXFE) == 0)
        ;
    // wait until not busy
    while ((UART1->FR & UART_FR_BUSY))
        ;
}

//! Uart0 for testing --------------------------------

//------------UART0_Init------------
// Initialize the UART for 115,200 baud rate (assuming 80 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART0_Init(void) {
    SYSCTL->RCGCUART |= 0x01;  // activate UART0
    SYSCTL->RCGCGPIO |= 0x01;  // activate port A
    while ((SYSCTL->PRGPIO & 0x01) == 0) {
    };
    UART0->CTL &= ~UART_CTL_UARTEN;  // disable UART
    UART0->IBRD = 325;               // IBRD = int(80,000,000 / (16 * 115200)) = int(43.402778)
    UART0->FBRD = 33;                // FBRD = round(0.402778 * 64) = 26
                                     // 8 bit word length (no parity bits, one stop bit, FIFOs)
    UART0->LCRH = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);

    UART0->IFLS &= ~0x3F;  // clear TX and RX interrupt FIFO level fields
                           // configure interrupt for TX FIFO <= 1/8 full
                           // configure interrupt for RX FIFO >= 1/8 full
    UART0->IFLS += (UART_IFLS_TX1_8 | UART_IFLS_RX1_8);
    // enable RX FIFO interrupts and RX time-out interrupt
    UART0->IM |= (UART_IM_RXIM | UART_IM_RTIM);
    UART0->CTL |= 0x301;   // enable UART
    GPIOA->AFSEL |= 0x03;  // enable alt funct on PA1-0
    GPIOA->DEN |= 0x03;    // enable digital I/O on PA1-0
                           // configure PA1-0 as UART
    GPIOA->PCTL = (GPIOA->PCTL & 0xFFFFFF00) + 0x00000011;
    GPIOA->AMSEL &= ~0x03;  // disable analog functionality on PA
}

//------------UART0_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
char UART0_InChar(void) {
    while ((UART0->FR & UART_FR_RXFE) != 0)
        ;
    // mask as it is a 32bit reg only need lower 8bits
    return ((char)(UART0->DR & 0xFF));  // type cast to char.
}
//------------UART0_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART0_OutChar(char data) {
    while ((UART0->FR & UART_FR_TXFF) != 0)
        ;
    UART0->DR = data;
}

//------------UART0_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART0_OutString(char *pt) {
    while (*pt) {
        UART0_OutChar(*pt);
        pt++;
    }
}

//------------UART0_InUDec------------
// InUDec accepts ASCII input in unsigned decimal format
//     and converts to a 32-bit unsigned number
//     valid range is 0 to 4294967295 (2^32-1)
// Input: none
// Output: 32-bit unsigned number
// If you enter a number above 4294967295, it will return an incorrect value
// Backspace will remove last digit typed
uint32_t UART0_InUDec(void) {
    uint32_t number = 0, length = 0;
    char character;
    character = UART0_InChar();
    while (character != CR_U) {  // accepts until <enter> is typed
                                 // The next line checks that the input is a digit, 0-9.
                                 // If the character is not 0-9, it is ignored and not echoed
        if ((character >= '0') && (character <= '9')) {
            number = 10 * number + (character - '0');  // this line overflows if above 4294967295
            length++;
            UART0_OutChar(character);
        }
        // If the input is a backspace, then the return number is
        // changed and a backspace is outputted to the screen
        else if ((character == BS) && length) {
            number /= 10;
            length--;
            UART0_OutChar(character);
        }
        character = UART0_InChar();
    }
    return number;
}

//-----------------------UART0_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART0_OutUDec(uint32_t n) {
    // This function uses recursion to convert decimal number
    //   of unspecified length as an ASCII string
    if (n >= 10) {
        UART0_OutUDec(n / 10);
        n = n % 10;
    }
    UART0_OutChar(n + '0'); /* n is between 0 and 9 */
}

//---------------------UART0_InUHex----------------------------------------
// Accepts ASCII input in unsigned hexadecimal (base 16) format
// Input: none
// Output: 32-bit unsigned number
// No '$' or '0x' need be entered, just the 1 to 8 hex digits
// It will convert lower case a-f to uppercase A-F
//     and converts to a 16 bit unsigned number
//     value range is 0 to FFFFFFFF
// If you enter a number above FFFFFFFF, it will return an incorrect value
// Backspace will remove last digit typed
uint32_t UART0_InUHex(void) {
    uint32_t number = 0;
    uint32_t digit = 0;
    uint32_t length = 0;

    char character;
    character = UART0_InChar();
    while (character != CR_U) {
        digit = 0x10;  // assume bad
        if ((character >= '0') && (character <= '9')) {
            digit = character - '0';
        } else if ((character >= 'A') && (character <= 'F')) {
            digit = (character - 'A') + 0xA;
        } else if ((character >= 'a') && (character <= 'f')) {
            digit = (character - 'a') + 0xA;
        }
        // If the character is not 0-9 or A-F, it is ignored and not echoed
        if (digit <= 0xF) {
            number = number * 0x10 + digit;
            length++;
            UART0_OutChar(character);
        }
        // Backspace outputted and return value changed if a backspace is inputted
        else if ((character == BS) && length) {
            number /= 0x10;
            length--;
            UART0_OutChar(character);
        }
        character = UART0_InChar();
    }
    return number;
}

//--------------------------UART0_OutUHex----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1 to 8 digits with no space before or after
void UART0_OutUHex(uint32_t number) {
    // This function uses recursion to convert the number of
    //   unspecified length as an ASCII string
    if (number >= 0x10) {
        UART0_OutUHex(number / 0x10);
        UART0_OutUHex(number % 0x10);
    } else {
        if (number < 0xA) {
            UART0_OutChar(number + '0');
        } else {
            UART0_OutChar((number - 0x0A) + 'A');
        }
    }
}
//--------------------------UART0_OutUHex2----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Fixed format 2 digits with no space before or after
void outnibble(uint32_t n) {
    if (n < 0xA) {
        UART0_OutChar(n + '0');
    } else {
        UART0_OutChar((n - 0x0A) + 'A');
    }
}
void UART0_OutUHex2(uint32_t number) {
    outnibble(number / 0x10);  // ms digit
    outnibble(number % 0x10);  // ls digit
}
//------------UART0_InString------------
// Accepts ASCII characters from the serial port
//    and adds them to a string until <enter> is typed
//    or until max length of the string is reached.
// It echoes each character as it is inputted.
// If a backspace is inputted, the string is modified
//    and the backspace is echoed
// terminates the string with a null character
// uses busy-waiting synchronization on RDRF
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
// -- Modified by Agustinus Darmawan + Mingjie Qiu --

void UART0_InString(char *bufPt, uint16_t max) {
    int length = 0;
    char character;
    character = UART0_InChar();
    while (character != CR_U) {
        if (character == BS) {
            if (length) {
                bufPt--;
                length--;
                UART0_OutChar(BS);
            }
        } else if (length < max) {
            *bufPt = character;
            bufPt++;
            length++;
            UART0_OutChar(character);
        }
        character = UART0_InChar();
    }
    *bufPt = 0;
}
/* End UART0_InString ============================================================================*/

/**
 * @brief Switch debouncing.
 * debounces all the system switches 
 * publishes the state of each switche to active objects that 
 * are subcribed to the switch events.
 */
static void l_switchDebounce(void) {
    /* state of the button debouncing, see below */
    static struct ButtonsDebouncing {
        uint32_t depressed;
        uint32_t previous;
    } buttons = {~0U, ~0U};
    uint32_t current;
    uint32_t tmp;
    // booster pack buttons
    //
    static struct ButtonsDebouncing1 {
        uint32_t depressed1;
        uint32_t previous1;
    } buttons1 = {~0U, ~0U};
    uint32_t current1;
    uint32_t swArray = 0;
    uint32_t tmp1 = 0;
    /* Perform the debouncing of buttons. The algorithm for debouncing
    * adapted from QP DPP example project by Miro Samek which was adapted from
    * the book "Embedded Systems Dictionary" by Jack Ganssle
    * and Michael Barr, page 71.
    */
    current = ~LEDS_PORT->DATA_Bits[BTN_SW1 | BTN_SW2];  // read SW1 and SW2
    tmp = buttons.depressed;                             // save the debounced depressed buttons
    buttons.depressed |= (buttons.previous & current);   // set depressed
    buttons.depressed &= (buttons.previous | current);   // clear released
    buttons.previous = current;                          // update the history
    tmp ^= buttons.depressed;                            // changed debounced depressed
    if ((tmp & BTN_SW1) != 0U) {                         // debounced SW1 state changed?
        if ((buttons.depressed & BTN_SW1) != 0U) {       // is SW1 depressed?
            static QEvt const btn1prsEvt = {BUTTON1_PRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&btn1prsEvt, &xHigherPriorityTaskWoken, &l_TickHook);
        } else {  // the button is released
            static QEvt const btn1DprsEvt = {BUTTON1_DEPRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&btn1DprsEvt, &xHigherPriorityTaskWoken, &l_TickHook);
        }
    }

    /* save current all current BoosterPack user switch states into single variable */
    swArray |= ((BSTR_BTN_SWS_PORT->DATA_Bits[BSTR_BTN_SW3]));
    swArray |= ((BSTR_BTN_SWS_PORT->DATA_Bits[BSTR_BTN_SW4]));
    swArray |= (BSP_BP_JST_BTN_PORT->DATA_Bits[BSP_BP_JST_BTN_PIN]);
    current1 = ~swArray;                                     // read SW1 and SW2
    tmp1 = buttons1.depressed1;                              // save the debounced depressed1 buttons1
    buttons1.depressed1 |= (buttons1.previous1 & current1);  // set depressed1
    buttons1.depressed1 &= (buttons1.previous1 | current1);  // clear released
    buttons1.previous1 = current1;                           // update the history
    tmp1 ^= buttons1.depressed1;                             // changed debounced depressed1
    if ((tmp1 & BSTR_BTN_SW3) != 0U) {                       // debounced SW1 state changed?
        if ((buttons1.depressed1 & BSTR_BTN_SW3) != 0U) {    // is SW1 depressed1?
            static QEvt const btn3prsEvt = {BUTTON3_PRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&btn3prsEvt, &xHigherPriorityTaskWoken, &l_TickHook);
        } else {  // the button is released
            static QEvt const btn3DprsEvt = {BUTTON3_DEPRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&btn3DprsEvt, &xHigherPriorityTaskWoken, &l_TickHook);
        }
    }

    if ((tmp1 & BSTR_BTN_SW4) != 0U) {                     // debounced SW1 state changed?
        if ((buttons1.depressed1 & BSTR_BTN_SW4) != 0U) {  // is SW depressed1?
            static QEvt const btn4prsEvt = {BUTTON4_PRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&btn4prsEvt, &xHigherPriorityTaskWoken, &l_TickHook);

        } else {  // the button is released
            static QEvt const btn4DeprsEvt = {BUTTON4_DEPRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&btn4DeprsEvt, &xHigherPriorityTaskWoken, &l_TickHook);
        }
    }

    //! debounce BoosterPack Joystick button
    if ((tmp1 & BSP_BP_JST_BTN_PIN) != 0U) {                     // debounced SW1 state changed?
        if ((buttons1.depressed1 & BSP_BP_JST_BTN_PIN) != 0U) {  // is SW depressed1?
            static QEvt const jstBtnPrsEvt = {JOYSTICK_PRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&jstBtnPrsEvt, &xHigherPriorityTaskWoken, &l_TickHook);
        } else {  // the button is released
            static QEvt const jstBtnDprsEvt = {JOYSTICK_DEPRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&jstBtnDprsEvt, &xHigherPriorityTaskWoken, &l_TickHook);
        }
    }
}
/* End l_switchDebounce =====================================================================*/


// DMA configuration ===============================================================
// pingpog mode
// sample length of 512 or start with 128
// 16 bit
// Two data arrays

// ADC PWM configuration ===========================================
// select pwm channel
// 500ms 10% duty cycle.


void BSP_ADC_PWM_Init(void) {
    
    SYSCTL->RCGCPWM |= 0x01;  // activate clock for pwm
    while ((SYSCTL->PRPWM & 0x01) == 0) {
    };  // allow time for clock to stabilize

    //M1PWM6------------------
    SYSCTL->RCC |= (0x00100000);  /* Enable System Clock Divisor function  */
    SYSCTL->RCC &= ~(0x000E0000); /* clear divide  */
    SYSCTL->RCC |= (0x00060000);  /* divide sysclk by 0002 0000 /2. 6 0000 /4. */
    // update the pwm clock freq divider used.
  
    PWM0->_0_CTL = 0;           // Disable Generator 0 counter
    PWM0->_0_CTL &= ~(1 << 1);  // select down count mode of counter 1
    // cont0ols when the output is enable and disabled according to Reload and CMPA values
    PWM0->_0_GENA |= (0x000000C0 | 0x00000008); /* Set PWM output when counter matches PWMCMPA clear on reload*/
    PWM0->_0_LOAD = 3125U;                      // set load value for 50Hz 16MHz/65 = 250kHz and (250KHz/5000)
    PWM0->_0_INTEN = 0x0A00;                    // Trigger ADC when counter matches CMPA
    //P0M1->_0_CMPA = 265; 10% duty register not configured, pwm output stays at 0.
    PWM0->_0_CTL |= 0x01;      // Enable Generator 3 counter
    PWM0->ENABLE |= (1U << 6); /* Enable PWM1 channel 0 output */
}
// start pwm trigger for ADC1
void BSP_ADC_PWM_Start(void){
    PWM0->_0_CMPA = 1562;  // 10 % duty 
}


//! helper function to be moved to own file
uint8_t friden_sqrt(uint32_t val) {
    uint32_t rem = 0;
    uint32_t root = 0;
    for (int i = 0; i < 16; i++) {
        root <<= 1;
        rem = ((rem << 2) + (val >> 30));
        val <<= 2;
        root++;
        if (root <= rem) {
            rem -= root;
            root++;
        } else
            root--;
    }
    return (uint8_t)(root >> 1);
}

uint32_t sqrt32(uint32_t s) {
    uint32_t t;             // t*t will become s
    int n;                  // loop counter
    t = s / 16 + 1;         // initial guess
    for (n = 16; n; --n) {  // will finish
        t = ((t * t + s) / t) / 2;
    }
    return t;
}