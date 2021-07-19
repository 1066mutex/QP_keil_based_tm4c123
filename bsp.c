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
    SYSTICK_PRIO = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8-__NVIC_PRIO_BITS)),
    GPIOA_PRIO,
    /* ... */
    MAX_KERNEL_AWARE_CMSIS_PRI /* keep always last */
};
/* "kernel-aware" interrupts should not overlap the PendSV priority */
Q_ASSERT_COMPILE(MAX_KERNEL_AWARE_CMSIS_PRI <= (0xFF >>(8-__NVIC_PRIO_BITS)));

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
    /* ... */

    /* enable IRQs... */
    NVIC_EnableIRQ(GPIOA_IRQn);

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
void BSP_Delay1ms(uint32_t n) {
    while (n) {
        parrotdelay(23746);  // 1 msec, tuned at 80 MHz, originally part of LCD module
        n--;
    }
}


// Initialization code common to both 'B' and 'R' type displays
void BSP_BP_SPI_TFT_Init(const uint8_t *cmdList) {
    //! ColStart = RowStart = 0;  // May be overridden in init func
    (void)cmdList; // todo not used should be removed.
    // toggle RST low to reset; CS low so it'll listen to us
    // SSI2Fss is not available, so use GPIO on PA4
    SYSCTL->RCGCGPIO |= 0x00000023;  // 1) activate clock for Ports F, B, and A
    while ((SYSCTL->PRGPIO & 0x23) != 0x23) {}; // allow time for clocks to stabilize
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

   // todo can make this into a function.
   
    BSP_BP_TFT_CS_LOW();
    BSP_BP_TFT_RESET_HIGH();
   // BSP_Delay1ms(100);
    BSP_BP_TFT_RESET_LOW();
   // BSP_Delay1ms(100);
    BSP_BP_TFT_RESET_HIGH();
   // BSP_Delay1ms(100);
    BSP_BP_TFT_CS_HIGH(); 
    
//=============================================================

    // TFT_CS = TFT_CS_LOW;
    // RESET = RESET_HIGH;
    // BSP_Delay1ms(500);
    // RESET = RESET_LOW;
    // BSP_Delay1ms(500);
    // RESET = RESET_HIGH;
    // BSP_Delay1ms(500);
    // TFT_CS = TFT_CS_HIGH;
// =============================================================
    // initialize SSI2
    // activate clock for SSI2
    SYSCTL->RCGCSSI |= 0x00000004;
    // allow time for clock to stabilize
    while ((SYSCTL->PRSSI & 0x00000004) == 0) {};
    SSI2->CR1 &= ~0x00000002;            // disable SSI
    SSI2->CR1 &= ~0x00000004;            // master mode
                                         // configure for clock from source PIOSC for baud clock source
    SSI2->CC = (SSI2->CC & ~0x0000000F) + 0x00000005;
                                        // clock divider for 4 MHz SSIClk (16 MHz PIOSC/4)
                                        // PIOSC/(CPSDVSR*(1+SCR))
                                        // 16/(4*(1+0)) = 4 MHz
    SSI2->CPSR = (SSI2->CPSR & ~0x000000FF) + 4; // must be even number
    SSI2->CR0 &= ~(0x0000FF00 |         // SCR = 0 (4 Mbps data rate)
                   0x00000080 |         // SPH = 0
                   0x00000040);         // SPO = 0
                                        // FRF = Freescale format
    SSI2->CR0 = (SSI2->CR0 & ~0x00000030) + 0x00000000;
                                        // DSS = 8-bit data
    SSI2->CR0 = (SSI2->CR0 & ~0x0000000F) + 0x00000007;
    SSI2->CR1 |= 0x00000002;             // enable SSI

   //!if (cmdList) commandList(cmdList); // removed
}

// BSP Booster pack LCD display functions======================================


