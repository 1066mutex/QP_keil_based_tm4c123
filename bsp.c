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
#include "dpp.h"
#include "bsp_bp.h"
#include "bsp.h"
#include "ports.h"

#include "TM4C123GH6PM.h"        /* the device specific header (TI) */
#include "rom.h"                 /* the built-in ROM functions (TI) */
#include "sysctl.h"              /* system control driver (TI) */
#include "gpio.h"                /* GPIO driver (TI) */
#include "hw_i2c.h"
/* add other drivers if necessary... */

#define TIMER_1A_PSV (TIMER1->TAPV) // TIMER prescaler value.
#define TIMER_1A_CLK  (SystemCoreClock/TIMER_1A_PSV) // timer 1A clk

#define PWM_CYCLES (SystemCoreClock / 2048) // time reload value for RGB (pwm) leds.

Q_DEFINE_THIS_FILE  /* define the name of this file for assertions */

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
* Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
* DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
*/
enum KernelUnawareISRs { /* see NOTE1 */
    UART0_PRIO,
    /* ... */
    MAX_KERNEL_UNAWARE_CMSIS_PRI /* keep always last */
};
/* "kernel-unaware" interrupts can't overlap "kernel-aware" interrupts */
Q_ASSERT_COMPILE(
   MAX_KERNEL_UNAWARE_CMSIS_PRI
   <= (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8-__NVIC_PRIO_BITS)));

enum KernelAwareISRs {
    SYSTICK_PRIO = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)),  //5
    GPIOA_PRIO = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)),    //5
    ADC0SS3_PRIO,  /* ADC0 SS3 priority */                                                                   
    /* ... */
    MAX_KERNEL_AWARE_CMSIS_PRI /* keep always last */

};
/* "kernel-aware" interrupts should not overlap the PendSV priority */
Q_ASSERT_COMPILE(MAX_KERNEL_AWARE_CMSIS_PRI <= (0xFF >> (8 - __NVIC_PRIO_BITS)));

/* booster pack LCD display.................................................*/
// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in ROM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80

/* Local-scope objects ------------------------------------------------------*/
static uint32_t l_rnd; // random seed

#ifdef Q_SPY

    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    /* QS identifiers for non-QP sources of events */
    static uint8_t const l_TickHook = (uint8_t)0;
    static uint8_t const l_GPIOPortA_IRQHandler = (uint8_t)0;

    #define UART_BAUD_RATE      115200U
    #define UART_FR_TXFE        (1U << 7)
    #define UART_FR_RXFE        (1U << 4)
    #define UART_TXFIFO_DEPTH   16U

    enum AppRecords { /* application-specific trace records */
        PHILO_STAT = QS_USER,
        PAUSED_STAT,
        COMMAND_STAT
    };

#endif
/* Local-scope functions ------------------------------------------------------*/
    void mcuTempSensorInit(void);

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
    void GPIOPortA_IRQHandler(void); /* prototype */
    void GPIOPortA_IRQHandler(void) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* for testing... */
        QACTIVE_POST_FROM_ISR(AO_Table,
                              Q_NEW_FROM_ISR(QEvt, MAX_PUB_SIG),
                              &xHigherPriorityTaskWoken,
                              &l_GPIOPortA_IRQHandler);

        /* the usual end of FreeRTOS ISR... */
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
/*..........................................................................*/
/* TODO: implement
* Move all my ISR to this section 
*/

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

/* Application hooks used in this project ==================================*/
/* NOTE: only the "FromISR" API variants are allowed in vApplicationTickHook */
void vApplicationTickHook(void) { //! called by freeRtos tick() 
    /* state of the button debouncing, see below */
    static struct ButtonsDebouncing {
        uint32_t depressed;
        uint32_t previous;
    } buttons = { ~0U, ~0U };
    uint32_t current;
    uint32_t tmp;
    // booster pack buttons
    //
    static struct ButtonsDebouncing1 {
        uint32_t depressed1;
        uint32_t previous1;
    } buttons1 = { ~0U, ~0U };
    uint32_t current1;
    uint32_t tmp1;
    
    // joystick button debouncing.
    static struct ButtonsDebouncing2 {
        uint32_t depressed1;
        uint32_t previous1;
    } buttons2 = { ~0U, ~0U };
    uint32_t current2;
    uint32_t tmp2;
    //! used to inform RTOS of a thread switch request.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* process time events for rate 0 */
    QF_TICK_X_FROM_ISR(0U, &xHigherPriorityTaskWoken, &l_TickHook);

#ifdef Q_SPY
    {
        tmp = SysTick->CTRL; /* clear SysTick_CTRL_COUNTFLAG */
        QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
    }
#endif
// TODO combine all the button debounce into one variable, get the state of each button
//      and pack into one variable then debounce them 

    /* Perform the debouncing of buttons. The algorithm for debouncing
    * adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
    * and Michael Barr, page 71.
    */
    current = ~LEDS_PORT->DATA_Bits[BTN_SW1 | BTN_SW2]; /* read SW1 and SW2 */
    tmp = buttons.depressed; /* save the debounced depressed buttons */
    buttons.depressed |= (buttons.previous & current); /* set depressed */
    buttons.depressed &= (buttons.previous | current); /* clear released */
    buttons.previous   = current; /* update the history */
    tmp ^= buttons.depressed;     /* changed debounced depressed */
    if ((tmp & BTN_SW1) != 0U) {  /* debounced SW1 state changed? */
        if ((buttons.depressed & BTN_SW1) != 0U) { /* is SW1 depressed? */
            /* demonstrate the ISR APIs: QF_PUBLISH_FROM_ISR and Q_NEW_FROM_ISR */
            QF_PUBLISH_FROM_ISR(Q_NEW_FROM_ISR(QEvt, PAUSE_SIG),
                                &xHigherPriorityTaskWoken,
                                &l_TickHook);
        }
        else { /* the button is released */
            /* demonstrate the ISR APIs: POST_FROM_ISR and Q_NEW_FROM_ISR */
            QACTIVE_POST_FROM_ISR(AO_Table,
                                  Q_NEW_FROM_ISR(QEvt, SERVE_SIG),
                                  &xHigherPriorityTaskWoken,
                                  &l_TickHook);
        }
    }
    //! debounce BoosterPack user buttons TODO: can select not to use the tiva buttons.
    current1 = ~BSTR_BTN_SWS_PORT->DATA_Bits[BSTR_BTN_SW3 | BSTR_BTN_SW4]; /* read SW1 and SW2 */
    tmp1 = buttons1.depressed1;                                              /* save the debounced depressed1 buttons1 */
    buttons1.depressed1 |= (buttons1.previous1 & current1);                    /* set depressed1 */
    buttons1.depressed1 &= (buttons1.previous1 | current1);                    /* clear released */
    buttons1.previous1 = current1;                                           /* update the history */
    tmp1 ^= buttons1.depressed1;                                             /* changed debounced depressed1 */
    if ((tmp1 & BSTR_BTN_SW3) != 0U) {                                     /* debounced SW1 state changed? */
        if ((buttons1.depressed1 & BSTR_BTN_SW3) != 0U) {                   /* is SW1 depressed1? */
            /* demonstrate the ISR APIs: QF_PUBLISH_FROM_ISR and Q_NEW_FROM_ISR */
            QF_PUBLISH_FROM_ISR(Q_NEW_FROM_ISR(QEvt, BUTTON3_PRESSED_SIG),
                                &xHigherPriorityTaskWoken,
                                &l_TickHook);
        } else { /* the button is released */
            /* demonstrate the ISR APIs: POST_FROM_ISR and Q_NEW_FROM_ISR */
            QACTIVE_POST_FROM_ISR(AO_Heartbeat, // BUG: this was set to AO_table
                                  Q_NEW_FROM_ISR(QEvt, BUTTON3_DEPRESSED_SIG),
                                  &xHigherPriorityTaskWoken,
                                  &l_TickHook);
        }
    }

    if ((tmp1 & BSTR_BTN_SW4) != 0U) {                   /* debounced SW1 state changed? */
        if ((buttons1.depressed1 & BSTR_BTN_SW4) != 0U) { /* is SW1 depressed1? */
            /* demonstrate the ISR APIs: QF_PUBLISH_FROM_ISR and Q_NEW_FROM_ISR */
            static QEvt const btn4prsEvt = {BUTTON4_PRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&btn4prsEvt, &xHigherPriorityTaskWoken, &l_TickHook);  // BUG: run QF_psInit()

        } else { /* the button is released */
            /* demonstrate the ISR APIs: POST_FROM_ISR and Q_NEW_FROM_ISR */
            static QEvt const btn4DeprsEvt = {BUTTON4_DEPRESSED_SIG, 0U, 0U};
            QF_PUBLISH_FROM_ISR(&btn4DeprsEvt, &xHigherPriorityTaskWoken, &l_TickHook);
        }
    }

    //! debounce BoosterPack Joystick button TODO: can select not to use the tiva buttons.
    current2 = ~BSP_BP_JST_BTN_PORT->DATA_Bits[BSP_BP_JST_BTN_PIN]; /* read SW1 and SW2 */
    tmp2 = buttons2.depressed1;                                              /* save the debounced depressed1 buttons1 */
    buttons2.depressed1 |= (buttons2.previous1 & current2);                    /* set depressed1 */
    buttons2.depressed1 &= (buttons2.previous1 | current2);                    /* clear released */
    buttons2.previous1 = current2;                                           /* update the history */
    tmp2 ^= buttons2.depressed1;                                             /* changed debounced depressed1 */
    if ((tmp2 & BSP_BP_JST_BTN_PIN) != 0U) {                                 /* debounced SW1 state changed? */
        if ((buttons2.depressed1 & BSP_BP_JST_BTN_PIN) != 0U) {              /* is SW1 depressed1? */
            /* demonstrate the ISR APIs: QF_PUBLISH_FROM_ISR and Q_NEW_FROM_ISR */
            QF_PUBLISH_FROM_ISR(Q_NEW_FROM_ISR(QEvt, JOYSTICK_PRESSED_SIG),
                                &xHigherPriorityTaskWoken,
                                &l_TickHook);
        } else { /* the button is released */
            /* demonstrate the ISR APIs: POST_FROM_ISR and Q_NEW_FROM_ISR */
            QACTIVE_POST_FROM_ISR(AO_Heartbeat, // BUG: this was set to AO_table
                                  Q_NEW_FROM_ISR(QEvt, JOYSTICK_DEPRESSED_SIG),
                                  &xHigherPriorityTaskWoken,
                                  &l_TickHook);
        }
    }
    /* notify FreeRTOS to perform context switch from ISR, if needed */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
/*..........................................................................*/
void vApplicationIdleHook(void) {
    float volatile x;

    /* toggle the User LED on and then off, see NOTE01 */
      QF_INT_DISABLE();
    GPIOF->DATA_Bits[LED_BLUE] = 0xFFU;  /* turn the Blue LED on  */
    GPIOF->DATA_Bits[LED_BLUE] = 0U;     /* turn the Blue LED off */
    QF_INT_ENABLE();

    /* Some flating point code is to exercise the VFP... */
    x = 1.73205F;
    x = x * 1.73205F;

#ifdef Q_SPY
    QS_rxParse();  /* parse all the received bytes */

    if ((UART0->FR & UART_FR_TXFE) != 0U) {  /* TX done? */
        uint16_t fifo = UART_TXFIFO_DEPTH;   /* max bytes we can accept */
        uint8_t const *block;

        QF_INT_DISABLE();
        block = QS_getBlock(&fifo);  /* try to get next block to transmit */
        QF_INT_ENABLE();

        while (fifo-- != 0) {  /* any bytes in the block? */
            UART0->DR = *block++;  /* put into the FIFO */
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
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside
    * this function then they must be declared static - otherwise they will
    * be allocated on the stack and so not exists after this function exits.
    */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

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
    SYSCTL->RCGCGPIO |= PORTD | PORTF ;

    /* configure the LEDs and push buttons */
    LEDS_PORT->DIR |= (LED_RED | LED_GREEN | LED_BLUE); /* set as output */
    LEDS_PORT->DEN |= (LED_RED | LED_GREEN | LED_BLUE); /* digital enable */
    LEDS_PORT->DATA_Bits[LED_RED]   = 0U; /* turn the LED off */
    LEDS_PORT->DATA_Bits[LED_GREEN] = 0U; /* turn the LED off */
    LEDS_PORT->DATA_Bits[LED_BLUE]  = 0U; /* turn the LED off */

    /* configure the User Switches on tm4c123 board */
    LEDS_PORT->DIR &= ~(BTN_SW1 | BTN_SW2); /*  set direction: input */
    ROM_GPIOPadConfigSet(GPIOF_BASE, (BTN_SW1 | BTN_SW2),
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* booster pack User Switches initialization */
    /* unlock gpio port D */
    BSTR_BTN_SWS_PORT->LOCK = 0x4C4F434B;
    BSTR_BTN_SWS_PORT->CR = 0xFF; /* allow changes to PD7-0 */
    BSTR_BTN_SWS_PORT->DIR &= ~(BSTR_BTN_SW3 | BSTR_BTN_SW4); /*  set direction: input */
   
    ROM_GPIOPadConfigSet(GPIOD_BASE, (BSTR_BTN_SW3 | BSTR_BTN_SW4),
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//BSP_BP_D_LedsInit();
BSP_BP_RGB_ledsInit();
BSP_BP_BuzzerIO_Init(200);
BSP_BP_Joystick_Init();
BSP_BP_SPI_TFT_Init();
BSP_Microphone_Init();
mcuTempSensorInit();
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

void BSP_BP_TFT_CS_LOW(void){
    BP_SPI_CS_PORT->DATA_Bits[BP_SPI_CS_PIN] = 0U;
}
//...........................................................................
void BSP_BP_TFT_CS_HIGH(void){
    BP_SPI_CS_PORT->DATA_Bits[BP_SPI_CS_PIN] = BP_SPI_CS_PIN;
}
//...........................................................................
void BSP_BP_TFT_RESET_HIGH(void){
    BP_LCD_RST_PORT->DATA_Bits[BP_LCD_RST_PIN] = BP_LCD_RST_PIN;
}
//...........................................................................
void BSP_BP_TFT_RESET_LOW(void){
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





// TODO: will not need this but a good example on toggling leds as display.
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    GPIOF->DATA_Bits[LED_RED] =
         ((stat[0] == 'e')   /* Is Philo[n] eating? */
         ? 0xFFU             /* turn the LED1 on  */
         : 0U);              /* turn the LED1 off */

    QS_BEGIN(PHILO_STAT, AO_Philo[n]) /* application-specific record begin */
        QS_U8(1, n);  /* Philosopher number */
        QS_STR(stat); /* Philosopher status */
    QS_END()
}
/*..........................................................................*/
void BSP_displayPaused(uint8_t paused) {
    GPIOF->DATA_Bits[LED_RED] = ((paused != 0U) ? 0xFFU : 0U);

    QS_BEGIN(PAUSED_STAT, (void *)0) /* application-specific record begin */
        QS_U8(1, paused);  /* Paused status */
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
    rnd = l_rnd * (3U*7U*11U*13U*23U);
    l_rnd = rnd; /* set for the next time */
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
    NVIC_SetPriority(UART0_IRQn,     UART0_PRIO);
    NVIC_SetPriority(SysTick_IRQn,   SYSTICK_PRIO);
    NVIC_SetPriority(GPIOA_IRQn,     GPIOA_PRIO);
    NVIC_SetPriority(ADC0SS3_IRQn, ADC0SS3_PRIO);
    /* ... */

    /* enable IRQs... */
    NVIC_EnableIRQ(GPIOA_IRQn);
    NVIC_EnableIRQ(ADC0SS3_IRQn);

#ifdef Q_SPY
    NVIC_EnableIRQ(UART0_IRQn);  /* UART0 interrupt used for QS-RX */
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
    while (GPIOF->DATA_Bits[BTN_SW1] != 0) {
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
    static uint8_t qsTxBuf[2*1024]; /* buffer for QS-TX channel */
    static uint8_t qsRxBuf[100];    /* buffer for QS-RX channel */
    uint32_t tmp;

    QS_initBuf  (qsTxBuf, sizeof(qsTxBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    /* enable clock for UART0 and GPIOA (used by UART0 pins) */
    SYSCTL->RCGCUART |= (1U << 0); /* enable Run mode for UART0 */
    SYSCTL->RCGCGPIO |= (1U << 0); /* enable Run mode for GPIOA */

    /* configure UART0 pins for UART operation */
    tmp = (1U << 0) | (1U << 1);
    GPIOA->DIR   &= ~tmp;
    GPIOA->SLR   &= ~tmp;
    GPIOA->ODR   &= ~tmp;
    GPIOA->PUR   &= ~tmp;
    GPIOA->PDR   &= ~tmp;
    GPIOA->AMSEL &= ~tmp;  /* disable analog function on the pins */
    GPIOA->AFSEL |= tmp;   /* enable ALT function on the pins */
    GPIOA->DEN   |= tmp;   /* enable digital I/O on the pins */
    GPIOA->PCTL  &= ~0x00U;
    GPIOA->PCTL  |= 0x11U;

    /* configure the UART for the desired baud rate, 8-N-1 operation */
    tmp = (((SystemCoreClock * 8U) / UART_BAUD_RATE) + 1U) / 2U;
    UART0->IBRD   = tmp / 64U;
    UART0->FBRD   = tmp % 64U;
    UART0->LCRH   = (0x3U << 5); /* configure 8-N-1 operation */
    UART0->LCRH  |= (0x1U << 4); /* enable FIFOs */
    UART0->CTL    = (1U << 0)    /* UART enable */
                    | (1U << 8)  /* UART TX enable */
                    | (1U << 9); /* UART RX enable */

    /* configure UART interrupts (for the RX channel) */
    UART0->IM   |= (1U << 4) | (1U << 6); /* enable RX and RX-TO interrupt */
    UART0->IFLS |= (0x2U << 2);    /* interrupt on RX FIFO half-full */
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
QSTimeCtr QS_onGetTime(void) {  /* NOTE: invoked with interrupts DISABLED */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { /* not set? */
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { /* the rollover occured, but the SysTick_ISR did not run yet */
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
                  uint32_t param1, uint32_t param2, uint32_t param3)
{
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
    }
    else if (cmdId == 11U) {
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
    // ***************** Timer1A initialization *****************
    SYSCTL->RCGCGPIO |= 0x0020;  // !activate clock for Port F DONE
    while ((SYSCTL->PRGPIO & 0x20) == 0) {
    };                           // allow time for clock to stabilize
    GPIOF->AFSEL |= 0x04;  // enable alt funct on PF2
    GPIOF->DEN |= 0x04;    // enable digital I/O on PF2
                                 // configure PF2 as PWM
    GPIOF->PCTL = (GPIOF->PCTL & 0xFFFFF0FF) + 0x00000500;
    GPIOF->AMSEL &= ~0x04;  // disable analog functionality on PF2
    
    SYSCTL->RCGCPWM |= 0x02;     // activate clock for Timer1
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
    PWM1->_3_CTL |= 0x01;                       // Enable Generator 3 counter
    PWM1->ENABLE |= (1U << 6);                  /* Enable PWM1 channel 0 output */

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


void BSP_BP_D_LedsInit(void){

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
    BSTR_LED_BLUE_PORT->AFSEL |= (BSTR_LED_BLUE);                                      /* disable alt funct on PC4 */
    BSTR_LED_BLUE_PORT->PUR &= ~(BSTR_LED_BLUE);                                        /* disable pull-up on PC4 */
    BSTR_LED_BLUE_PORT->PCTL |= ((BSTR_LED_BLUE_PORT->PCTL & 0xFFF0FFFF) + 0x00070000); /* configure PC4 as PWM */
    BSTR_LED_BLUE_PORT->AMSEL &= ~(BSTR_LED_BLUE);                                      /* disable analog functionality */
    BSTR_LED_BLUE_PORT->DEN |= (BSTR_LED_BLUE);                                         /* digital enable */
    // PF3 for Timer1B
    BSTR_LED_RED_PORT->DIR |= (BSTR_LED_RED);                                         /* set as output */
    BSTR_LED_RED_PORT->AFSEL |= (BSTR_LED_RED);                                      /* disable alt funct on PF3 */
    BSTR_LED_RED_PORT->PUR &= ~(BSTR_LED_RED);                                        /* disable pull-up on PF3 */
    BSTR_LED_RED_PORT->PCTL |= ((BSTR_LED_RED_PORT->PCTL & 0xFFFF0FFF) + 0x00007000); /* configure PF3 as PWM */
    BSTR_LED_RED_PORT->AMSEL &= ~(BSTR_LED_RED);                                      /* disable analog functionality */
    BSTR_LED_RED_PORT->DEN |= (BSTR_LED_RED);                                       /* digital enable */

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
    TIMER1->TBILR = PWM_CYCLES;                // defines when output signal is set
    //TIMER1->TBMATCHR register is not set, keeps pwm output at 0
    //
    // enable Timer1B 16-b, PWM, inverted to match comments
    TIMER1->CTL |= (0x00004000 | 0x00000100);  
    //==============================================================================================  

    // ***************** Timer3B (PB3) initialization *****************
    SYSCTL->RCGCTIMER |= 0x08;  // activate clock for Timer1

    while ((SYSCTL->PRTIMER & 0x08) == 0) {};   // allow time for clock to stabilize
    TIMER3->CTL &= ~(0x00000100);               // disable Timer3B during setup
    TIMER3->CFG = 0x00000004;                   // configure for 16-bit timer mode
                                                // configure for alternate (PWM) mode
    TIMER3->TBMR = (0x00000008 | 0x00000002);
    TIMER3->TBILR = PWM_CYCLES;                  // defines when output signal is set
    //TIMER3->TBMATCHR register is not set, keeps pwm output at 0
    //
    // enable Timer1B 16-b, PWM, inverted to match comments
    TIMER3->CTL |= (0x00004000 | 0x00000100);   // enable PWM

    //==============================================================================================

    // ***************** Wide Timer0A (PC4) initialization *****************
    SYSCTL->RCGCWTIMER |= 0x01;  // activate clock for Wide Timer0

    while ((SYSCTL->PRWTIMER & 0x01) == 0) {};  // allow time for clock to stabilize
    WTIMER0->CTL &= ~(0x0000001);               // disable Timer3B during setup
    WTIMER0->CFG = 0x00000004;                  // configure for 32-bit timer mode
                                                // configure for alternate (PWM) mode
    WTIMER0->TAMR = (0x00000008 | 0x00000002);
    WTIMER0->TAILR = PWM_CYCLES;                 // defines when output signal is set
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
void BSP_BP_LedRedDuty(uint8_t dutyCycle){
    if (dutyCycle > 100) {
        /* duty cycle is out of range do nothing */
        return; // todo: can assert here.
    }
    TIMER1->TBMATCHR = ((PWM_CYCLES*dutyCycle)/100)-1;
}
void BSP_BP_LedGreenDuty(uint8_t dutyCycle){
    if (dutyCycle > 100) {
        /* duty cycle is out of range do nothing */
        return; // todo: can assert here.
    }
    TIMER3->TBMATCHR = ((PWM_CYCLES*dutyCycle)/100)-1;
}
void BSP_BP_LedBlueDuty(uint8_t dutyCycle){
    if (dutyCycle > 100) {
        /* duty cycle is out of range do nothing */
        return; // todo: can assert here.
    }
    WTIMER0->TAMATCHR = ((PWM_CYCLES*dutyCycle)/100)-1;
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
    while ((SSI2->SR & 0x00000010) == 0x00000010) {};
    BSP_BP_TFT_CS_LOW();
    BSP_BP_TFT_DC_COMMAND();
    SSI2->DR = c;  // data out
    while ((SSI2->SR & 0x00000004) == 0) {};  // wait until response
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
    while ((SSI2->SR & 0x00000010) == 0x00000010) {};
    BSP_BP_TFT_CS_LOW();
    BSP_BP_TFT_DC_DATA();
    SSI2->DR = c;  // data out
    while ((SSI2->SR & 0x00000004) == 0) {};  // wait until response
    BSP_BP_TFT_CS_HIGH();
    return (uint8_t)SSI2->DR;  // return the response
}

//! delay function required for initialisation of LCD display.
//BUG: not sure if this will work with the arm none eabi gcc compiler!
// try the TI compiler version  

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
    subs    r0, #1 
    bne     parrotdelay
    bx      lr
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

// ADC initialisation==========================================================//
// There are six analog inputs on the Educational BoosterPack MKII:
// microphone (J1.6/PE5/AIN8)
// joystick X (J1.2/PB5/AIN11) and Y (J3.26/PD3/AIN4)
// accelerometer X (J3.23/PD0/AIN7), Y (J3.24/PD1/AIN6), and Z (J3.25/PD2/AIN5)
// All six initialization functions can use this general ADC
// initialization.  The joystick uses sample sequencer 1,
// the accelerometer sample sequencer 2, and the microphone
// uses sample sequencer 3.
static void adcinit(void) {
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

    adcinit();  // 8-9) general ADC initialization

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
       
    *x = ADC0->SSFIFO1 >> 2;  // read first result

    *y = ADC0->SSFIFO1 >> 2;  // 3b) read second result
    ADC0->ISC = 0x0002;       // 4) acknowledge completion
}

// trigger joystick ADC
void BSP_Joystick_Trigger(void) {
    ADC0->PSSI |= (1 << 1); /* Enable SS2 conversion or start sampling data from AN0 */
}

// ADC initialization for onboard temp sensor.
// Using intrrupt mode.
static void mcuTempSensorInit(void) {
    SYSCTL->RCGCADC |= 1; /* enable clock to ADC0 */
    /* initialize ADC0 */
    ADC0->ACTSS &= ~8;         /* disable SS3 during configuration */
    ADC0->EMUX &= ~0xF000;     /* software trigger conversion seq 0 */
    ADC0->SSMUX3 = 0;          /* get input from channel 0 */
    ADC0->SSCTL3 |= 0x0E;      /* take chip temperature, set flag at 1st sample */
    ADC0->IM |= 0x0008;        //  enable SS3 interrupts
    ADC0->ACTSS |= (1U << 3);  //  enable sample sequencer 3
}
// TODO place in the isr section above.
void ADCSeq3_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    ADC0->ISC |= (1U << 3);  // clear interrupt flag

    TempEvt *reading = Q_NEW_FROM_ISR(TempEvt, NEW_TEMP_DATA_SIG);

    // read adc coversion result from SS3 FIFO and convert to °C using
    // Temp = 147.5 – ((75 × Vref(+) – Vref(–)) × ADC_output)) / 4096. tm4c vref == 3.3v
    //
    reading->temp = 147 - ((247 * ADC0->SSFIFO3) / 4096);
    QACTIVE_POST_FROM_ISR(AO_Heartbeat,
                          (QEvt *)reading,
                          &xHigherPriorityTaskWoken,
                          &ADCSeq3_IRQHandler);

    /* the usual end of FreeRTOS ISR... */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

// trigger the ADC to sample the MCU temperature sensor.
void BSP_SystemTempGet(void) {
    ADC0->PSSI |= (1 << 3); /* Enable SS3 conversion or start sampling data from AN0 */
}

// ------------BSP_Accelerometer_Init------------
// Initialize three ADC pins, which correspond with
// BoosterPack pins J3.23 (X), J3.24 (Y), and
// J3.25 (Y).
// Input: none
// Output: none
void BSP_Accelerometer_Init(void) {
    adcinit();
    SYSCTL->RCGCGPIO |= 0x00000008;  // 1) activate clock for Port D
    while ((SYSCTL->PRGPIO & 0x08) == 0) {
    };                           // allow time for clock to stabilize
                                 // 2) no need to unlock PD2-0
    GPIOD->AMSEL |= 0x07;  // 3) enable analog on PD2-0
                                 // 4) configure PD2-0 as ?? (skip this line because PCTL is for digital only)
    GPIOD->DIR &= ~0x07;   // 5) make PD2-0 input
    GPIOD->AFSEL |= 0x07;  // 6) enable alt funct on PD2-0
    GPIOD->DEN &= ~0x07;   // 7) enable analog functionality on PD2-0
    adcinit();                   // 8-9) general ADC initialization
    ADC0->ACTSS &= ~0x0004;     // 10) disable sample sequencer 2
    ADC0->EMUX &= ~0x0F00;      // 11) seq2 is software trigger
    ADC0->SSMUX2 = 0x0567;      // 12) set channels for SS2
    ADC0->SSCTL2 = 0x0600;      // 13) no D0 END0 IE0 TS0 D1 END1 IE1 TS1 D2 TS2, yes IE2 END2
    ADC0->IM &= ~0x0004;        // 14) disable SS2 interrupts
    ADC0->ACTSS |= 0x0004;      // 15) enable sample sequencer 2
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
void BSP_Accelerometer_Input(uint16_t *x, uint16_t *y, uint16_t *z) {
    ADC0->PSSI = 0x0004;  // 1) initiate SS2
    while ((ADC0->RIS & 0x04) == 0) {};// 2) wait for conversion done
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

// ------------BSP_Microphone_Init------------
// Initialize one ADC pin, which corresponds with
// BoosterPack pin J1.6. tm4c123 pin PE5.
// Input: none
// Output: none
void BSP_Microphone_Init(void) {
    adcinit();
    SYSCTL->RCGCGPIO |= 0x00000010;  // 1) activate clock for Port E
    while ((SYSCTL->PRGPIO & 0x10) == 0) {
    };                           // allow time for clock to stabilize
                                 // 2) no need to unlock PE5
    GPIOE->AMSEL |= 0x20;  // 3) enable analog on PE5
                                 // 4) configure PE5 as ?? (skip this line because PCTL is for digital only)
    GPIOE->DIR &= ~0x20;   // 5) make PE5 input
    GPIOE->AFSEL |= 0x20;  // 6) enable alt funct on PE5
    GPIOE->DEN &= ~0x20;   // 7) enable analog functionality on PE5
    adcinit();                   // 8-9) general ADC initialization
    ADC0->ACTSS &= ~0x0008;     // 10) disable sample sequencer 3
    ADC0->EMUX &= ~0xF000;      // 11) seq3 is software trigger
    ADC0->SSMUX3 = 0x0008;      // 12) set channels for SS3
    ADC0->SSCTL3 = 0x0006;      // 13) no D0 TS0, yes IE0 END0
    ADC0->IM &= ~0x0008;        // 14) disable SS3 interrupts
    ADC0->ACTSS |= 0x0008;      // 15) enable sample sequencer 3
}

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
    *mic = ADC0->SSFIFO3 >> 2;  // 3) read result
    ADC0->ISC = 0x0008;         // 4) acknowledge completion
}

// trigger the ADC to sample the mic input.
void BSP_Microphone_Get(void){
    ADC0->PSSI = 0x0008;  // 1) initiate SS3
}


// I2C initialization==============================================

// There are two I2C devices on the Educational BoosterPack MKII:
// OPT3001 Light Sensor
// !TMP006 Temperature sensor not populated.
// Both initialization functions can use this general I2C
// initialization.
#define MAXRETRIES 5  // number of receive attempts before giving up
void static i2cinit(void) {
    SYSCTL->RCGCI2C |= 0x0002;   // 1a) activate clock for I2C1
    SYSCTL->RCGCGPIO |= PORTA;   // 1b) activate clock for Port A
    while ((SYSCTL->PRGPIO & 0x01) == 0) {
    };                            // allow time for clock to stabilize
                                  // 2) no need to unlock PA7-6
    GPIOA->AMSEL &= ~0xC0;        // 3) disable analog functionality on PA7-6
                                  // 4) configure PA7-6 as I2C1
    GPIOA->PCTL = (GPIOA->PCTL & 0x00FFFFFF) + 0x33000000;
    GPIOA->ODR |= 0x80;           // 5) enable open drain on PA7 only
    GPIOA->AFSEL |= 0xC0;         // 6) enable alt funct on PA7-6
    GPIOA->DEN |= 0xC0;           // 7) enable digital I/O on PA7-6
    I2C1->MCR = 0x00000010;       // 8) master function enable
    I2C1->MTPR = 39;              // 9) configure for 100 kbps clock
                                  // 20*(TPR+1)*12.5ns = 10us, with TPR=39

}

// receives two bytes from specified slave
// Note for HMC6352 compass only:
// Used with 'A' commands
// Note for TMP102 thermometer only:
// Used to read the contents of the pointer register
uint16_t static I2C_Recv2(int8_t slave) {
    uint8_t data1, data2;
    int retryCounter = 1;
    do {
        while (I2C1->MCS & I2C_MCS_BUSY) {
        };                                 // wait for I2C ready
        I2C1->MSA = (slave << 1) & 0xFE;  // MSA[7:1] is slave address
        I2C1->MSA |= 0x01;                // MSA[0] is 1 for receive
        I2C1->MCS = (0 | I2C_MCS_ACK      // positive data ack
                                           //                         & ~I2C_MCS_STOP    // no stop
                      | I2C_MCS_START      // generate start/restart
                      | I2C_MCS_RUN);      // master enable
        while (I2C1->MCS & I2C_MCS_BUSY) {
        };                            // wait for transmission done
        data1 = (I2C1->MDR & 0xFF);  // MSB data sent first
        I2C1->MCS = (0
                      //                         & ~I2C_MCS_ACK     // negative data ack (last byte)
                      | I2C_MCS_STOP   // generate stop
                                       //                         & ~I2C_MCS_START   // no start/restart
                      | I2C_MCS_RUN);  // master enable
        while (I2C1->MCS & I2C_MCS_BUSY) {
        };                                // wait for transmission done
        data2 = (I2C1->MDR & 0xFF);      // LSB data sent last
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
uint16_t static I2C_Send1(int8_t slave, uint8_t data1) {
    while (I2C1->MCS & I2C_MCS_BUSY) {
    };                                 // wait for I2C ready
    I2C1->MSA = (slave << 1) & 0xFE;  // MSA[7:1] is slave address
    I2C1->MSA &= ~0x01;               // MSA[0] is 0 for send
    I2C1->MDR = data1 & 0xFF;         // prepare first byte
    I2C1->MCS = (0
                  //                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                  | I2C_MCS_STOP   // generate stop
                  | I2C_MCS_START  // generate start/restart
                  | I2C_MCS_RUN);  // master enable
    while (I2C1->MCS & I2C_MCS_BUSY) {
    };  // wait for transmission done
        // return error bits
    return (I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR));
}

// sends three bytes to specified slave
// Note for HMC6352 compass only:
// Used with 'w' and 'G' commands
// Note for TMP102 thermometer only:
// Used to change the contents of the pointer register
// Returns 0 if successful, nonzero if error
uint16_t static I2C_Send3(int8_t slave, uint8_t data1, uint8_t data2, uint8_t data3) {
    while (I2C1->MCS & I2C_MCS_BUSY) {
    };                                 // wait for I2C ready
    I2C1->MSA = (slave << 1) & 0xFE;  // MSA[7:1] is slave address
    I2C1->MSA &= ~0x01;               // MSA[0] is 0 for send
    I2C1->MDR = data1 & 0xFF;         // prepare first byte
    I2C1->MCS = (0
                  //                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                  //                       & ~I2C_MCS_STOP    // no stop
                  | I2C_MCS_START  // generate start/restart
                  | I2C_MCS_RUN);  // master enable
    while (I2C1->MCS & I2C_MCS_BUSY) {
    };  // wait for transmission done
        // check error bits
    if ((I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR)) != 0) {
        I2C1->MCS = (0               // send stop if nonzero
                                      //                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                      | I2C_MCS_STOP  // stop
                                      //                       & ~I2C_MCS_START   // no start/restart
                                      //                       & ~I2C_MCS_RUN     // master disable
        );
        // return error bits if nonzero
        return (I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR));
    }
    I2C1->MDR = data2 & 0xFF;  // prepare second byte
    I2C1->MCS = (0
                  //                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                  //                       & ~I2C_MCS_STOP    // no stop
                  //                       & ~I2C_MCS_START   // no start/restart
                  | I2C_MCS_RUN);  // master enable
    while (I2C1->MCS & I2C_MCS_BUSY) {
    };  // wait for transmission done
        // check error bits
    if ((I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR)) != 0) {
        I2C1->MCS = (0               // send stop if nonzero
                                      //                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                      | I2C_MCS_STOP  // stop
                                      //                       & ~I2C_MCS_START   // no start/restart
                                      //                       & ~I2C_MCS_RUN   // master disable
        );
        // return error bits if nonzero
        return (I2C1->MCS & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR));
    }
    I2C1->MDR = data3 & 0xFF;  // prepare third byte
    I2C1->MCS = (0
                  //                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                  | I2C_MCS_STOP   // generate stop
                                   //                       & ~I2C_MCS_START   // no start/restart
                  | I2C_MCS_RUN);  // master enable
    while (I2C1->MCS & I2C_MCS_BUSY) {
    };  // wait for transmission done
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
void BSP_LightSensor_Init(void) {
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

// Send the appropriate codes to initiate a
// measurement with an OPT3001 light sensor at I2C
// slave address 'slaveAddress'.
// Assumes: BSP_LightSensor_Init() has been called
void static lightsensorstart(uint8_t slaveAddress) {
    // configure Low Limit Register (0x02) for:
    // INT pin active after each conversion completes
    I2C_Send3(slaveAddress, 0x02, 0xC0, 0x00);
    // configure Configuration Register (0x01) for:
    // 15-12 RN         range number         1100b = automatic full-scale setting mode
    // 11    CT         conversion time         1b = 800 ms
    // 10-9  M          mode of conversion     01b = single-shot
    // 8     OVF        overflow flag field     0b (read only)
    // 7     CRF        conversion ready field  0b (read only)
    // 6     FH         flag high field         0b (read only)
    // 5     FL         flag low field          0b (read only)
    // 4     L          latch                   1b = latch interrupt if measurement exceeds programmed ranges
    // 3     POL        polarity                0b = INT pin reports active low
    // 2     ME         mask exponent           0b = do not mask exponent (more math later)
    // 1-0   FC         fault count            00b = 1 fault triggers interrupt
    I2C_Send3(slaveAddress, 0x01, 0xCA, 0x10);
    I2C_Recv2(slaveAddress);  // read Configuration Register to reset conversion ready
}

// Send the appropriate codes to end a measurement
// with an OPT3001 light sensor at I2C slave address
// 'slaveAddress'.  Return results (units 100*lux).
// Assumes: BSP_LightSensor_Init() has been called and measurement is ready
int32_t static lightsensorend(uint8_t slaveAddress) {
    uint16_t raw, config;
    I2C_Send1(slaveAddress, 0x00);  // pointer register 0x00 = Result Register
    raw = I2C_Recv2(slaveAddress);
    // force the INT pin to clear by clearing and resetting the latch bit of the Configuration Register (0x01)
    I2C_Send1(slaveAddress, 0x01);     // pointer register 0x01 = Configuration Register
    config = I2C_Recv2(slaveAddress);  // current Configuration Register
    I2C_Send3(slaveAddress, 0x01, (config & 0xFF00) >> 8, (config & 0x00FF) & ~0x0010);
    I2C_Send3(slaveAddress, 0x01, (config & 0xFF00) >> 8, (config & 0x00FF) | 0x0010);
    return (1 << (raw >> 12)) * (raw & 0x0FFF);
}

// ------------BSP_LightSensor_Input------------
// Query the OPT3001 light sensor for a measurement.
// Wait until the measurement is ready, which may
// take 800 ms.
// Input: none
// Output: light intensity (units 100*lux)
// Assumes: BSP_LightSensor_Init() has been called
#define LIGHTINT (*((volatile uint32_t *)0x40004080)) /* PA5 */
int LightBusy = 0;                                    // 0 = idle; 1 = measuring
uint32_t BSP_LightSensor_Input(void) {
    uint32_t light;
    LightBusy = 1;
    lightsensorstart(0x44);
    while (LIGHTINT == 0x20) {
    };  // wait for conversion to complete
    light = lightsensorend(0x44);
    LightBusy = 0;
    return light;
}

// ------------BSP_LightSensor_Start------------
// Start a measurement using the OPT3001.
// If a measurement is currently in progress, return
// immediately.
// Input: none
// Output: none
// Assumes: BSP_LightSensor_Init() has been called
void BSP_LightSensor_Start(void) {
    if (LightBusy == 0) {
        // no measurement is in progress, so start one
        LightBusy = 1;
        lightsensorstart(0x44);
    }
}

// ------------BSP_LightSensor_End------------
// Query the OPT3001 light sensor for a measurement.
// If no measurement is currently in progress, start
// one and return zero immediately.  If the measurement
// is not yet complete, return zero immediately.  If
// the measurement is complete, store the result in the
// pointer provided and return one.
// Input: light is pointer to store light intensity (units 100*lux)
// Output: one if measurement is ready and pointer is valid
//         zero if measurement is not ready and pointer unchanged
// Assumes: BSP_LightSensor_Init() has been called
int BSP_LightSensor_End(uint32_t *light) {
    uint32_t lightLocal;
    if (LightBusy == 0) {
        // no measurement is in progress, so start one
        LightBusy = 1;
        lightsensorstart(0x44);
        return 0;  // measurement needs more time to complete
    } else {
        // measurement is in progress
        if (LIGHTINT == 0x20) {
            return 0;  // measurement needs more time to complete
        } else {
            lightLocal = lightsensorend(0x44);
            *light = lightLocal;
            LightBusy = 0;
            return 1;  // measurement is complete; pointer valid
        }
    }
}


// ------------BSP_TempSensor_Init------------
// Initialize a GPIO pin for input, which corresponds
// with BoosterPack pins J2.11 (Temperature Sensor
// interrupt).  Initialize two I2C pins, which
// correspond with BoosterPack pins J1.9 (SCL) and
// J1.10 (SDA).
// Input: none
// Output: none
void BSP_TempSensor_Init(void) {
    i2cinit();
    // 1) activate clock for Port A (done in i2cinit())
    // allow time for clock to stabilize (done in i2cinit())
    // 2) no need to unlock PA2
    GPIOA->AMSEL &= ~0x04;  // 3) disable analog on PA2
                                  // 4) configure PA2 as GPIO
    GPIOA->PCTL = (GPIOA->PCTL & 0xFFFFF0FF) + 0x00000000;
    GPIOA->DIR &= ~0x04;    // 5) make PA5 input
    GPIOA->AFSEL &= ~0x04;  // 6) disable alt funct on PA2
    GPIOA->DEN |= 0x04;     // 7) enable digital I/O on PA2
}

// Send the appropriate codes to initiate a
// measurement with a TMP006 temperature sensor at
// I2C slave address 'slaveAddress'.
// Assumes: BSP_TempSensor_Init() has been called
void static tempsensorstart(uint8_t slaveAddress) {
    // configure Configuration Register (0x02) for:
    // 15    RST        software reset bit      0b = normal operation
    // 14-12 MOD        mode of operation     111b = sensor and die continuous conversion
    // 11-9  CR         ADC conversion rate   010b = 1 sample/sec
    // 8     EN         interrupt pin enable    1b = ~DRDY pin enabled (J2.11/P3.6)
    // 7     ~DRDY      data ready bit          0b (read only, automatic clear)
    // 6-0   reserved                      000000b (reserved)
    I2C_Send3(slaveAddress, 0x02, 0x75, 0x00);
}

// Send the appropriate codes to end a measurement
// with a TMP006 temperature sensor at I2C slave
// address 'slaveAddress'.  Store the results at the
// provided pointers.
// Assumes: BSP_TempSensor_Init() has been called and measurement is ready
void static tempsensorend(uint8_t slaveAddress, int32_t *sensorV, int32_t *localT) {
    int16_t raw;
    I2C_Send1(slaveAddress, 0x00);  // pointer register 0x00 = Sensor Voltage Register
    raw = I2C_Recv2(slaveAddress);
    *sensorV = raw * 15625;         // 156.25 nV per LSB
    I2C_Send1(slaveAddress, 0x01);  // pointer register 0x01 = Local Temperature Register
    raw = I2C_Recv2(slaveAddress);
    *localT = (raw >> 2) * 3125;  // 0.03125 C per LSB
}

// ------------BSP_TempSensor_Input------------
// Query the TMP006 temperature sensor for a
// measurement.  Wait until the measurement is ready,
// which may take 4 seconds.
// Input: sensorV is signed pointer to store sensor voltage (units 100*nV)
//        localT is signed pointer to store local temperature (units 100,000*C)
// Output: none
// Assumes: BSP_TempSensor_Init() has been called
#define TEMPINT (*((volatile uint32_t *)0x40004010)) /* PA2 */
int TempBusy = 0;                                    // 0 = idle; 1 = measuring
void BSP_TempSensor_Input(int32_t *sensorV, int32_t *localT) {
    int32_t volt, temp;
    TempBusy = 1;
    tempsensorstart(0x40);
    while (TEMPINT == 0x04) {
    };  // wait for conversion to complete
    tempsensorend(0x40, &volt, &temp);
    *sensorV = volt;
    *localT = temp;
    TempBusy = 0;
}

// ------------BSP_TempSensor_Start------------
// Start a measurement using the TMP006.
// If a measurement is currently in progress, return
// immediately.
// Input: none
// Output: none
// Assumes: BSP_TempSensor_Init() has been called
void BSP_TempSensor_Start(void) {
    if (TempBusy == 0) {
        // no measurement is in progress, so start one
        TempBusy = 1;
        tempsensorstart(0x40);
    }
}

// ------------BSP_TempSensor_End------------
// Query the TMP006 temperature sensor for a
// measurement.  If no measurement is currently in
// progress, start one and return zero immediately.
// If the measurement is not yet complete, return
// zero immediately.  If the measurement is complete,
// store the result in the pointers provided and
// return one.
// Input: sensorV is signed pointer to store sensor voltage (units 100*nV)
//        localT is signed pointer to store local temperature (units 100,000*C)
// Output: one if measurement is ready and pointers are valid
//         zero if measurement is not ready and pointers unchanged
// Assumes: BSP_TempSensor_Init() has been called
int BSP_TempSensor_End(int32_t *sensorV, int32_t *localT) {
    int32_t volt, temp;
    if (TempBusy == 0) {
        // no measurement is in progress, so start one
        TempBusy = 1;
        tempsensorstart(0x40);
        return 0;  // measurement needs more time to complete
    } else {
        // measurement is in progress
        if (TEMPINT == 0x04) {
            return 0;  // measurement needs more time to complete
        } else {
            tempsensorend(0x40, &volt, &temp);
            *sensorV = volt;
            *localT = temp;
            TempBusy = 0;
            return 1;  // measurement is complete; pointers valid
        }
    }
}
