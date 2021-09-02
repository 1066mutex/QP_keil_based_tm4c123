/**
 * @file   blinky.c 
 * @author Eddie Mhako (eddie.mhako@sky.com)
 * @brief  Blinky active object is used for testing purposes, it manages the UI interface
 *         LCD display, LEDs and buzzer.  
 * @version 0.1
 * @date   2021-08-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "qpc.h"
#include "bsp.h"
#include "bsp_bp.h"
#include "dpp.h"

/* Active object class  ############################################*/

// AO_BLE_uart =======

typedef struct
{  // protected
    QActive super;
    // private

    uint8_t tempAvgInx;
    uint16_t urtCharToInt;
    uint16_t ledPwm;
    uint16_t urtIdx;
    char urtFrame[20];
    enum { WAITING = 1,
           GOT_START
    } urtFrameState;

} BLE_uart;

/* protected: */
static QState BLE_uart_initial(BLE_uart* const me, QEvt const* const e);
static QState BLE_uart_packing(BLE_uart* const me, QEvt const* const e);
static QState BLE_uart_start(BLE_uart* const me, QEvt const* const e);
static QState BLE_uart_off(BLE_uart* const me, QEvt const* const e);
//============================================================

// Local instance of BLE_uart object===============================
//
static BLE_uart l_BLE_uart;
/* Global-scope objects ----------------------------------------------------*/
QActive* const AO_BLE_uart = &l_BLE_uart.super;

// AO_BLE_uart constructor==================================
//
void BLE_uart_ctor(void) {
    BLE_uart* me = (BLE_uart*)AO_BLE_uart;
    // add AO to qp
    //
    QActive_ctor(&me->super, Q_STATE_CAST(&BLE_uart_initial));
    //QTimeEvt_ctorX(&me->timeEvt, &me->super, TIMEOUT1_SIG, 0U);
}

static QState BLE_uart_initial(BLE_uart* const me, QEvt const* const e) {
    (void)e;
    // QS_OBJ_DICTIONARY(&l_BLE_uart);
    // QS_FUN_DICTIONARY(&QHsm_top);
    // QS_FUN_DICTIONARY(&BLE_uart_initial);
    // QS_FUN_DICTIONARY(&BLE_uart_off);
    // QS_FUN_DICTIONARY(&BLE_uart_on);
    // Q spy dummy code.

    QS_SIG_DICTIONARY(BUTTON4_DEPRESSED_SIG, (void*)0);
    QS_SIG_DICTIONARY(BUTTON4_PRESSED_SIG, (void*)0);

    QActive_subscribe(&me->super, BUTTON4_DEPRESSED_SIG);
    QActive_subscribe(&me->super, BUTTON4_PRESSED_SIG);

    me->urtIdx = 0;
    me->urtFrameState = WAITING;     // uart frame state machine start state.
    return Q_TRAN(&BLE_uart_start);  //*go to off (1)
}
static QState BLE_uart_start(BLE_uart* const me, QEvt const* const e) {
    // super state for on and off states

    QState status_;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            status_ = Q_HANDLED();  //*do entry actions (3).
            break;
        }

        case Q_EXIT_SIG: {
            status_ = Q_HANDLED();
            break;
        }

        case NEW_UART_DATA_SIG: {
            
            UartEvt const* evt = Q_EVT_CAST(UartEvt);
            uint8_t n;
            for (n = 0; (n < evt->len); ++n) {
                UART0_OutChar(evt->chars[n]);
            }

            //UART0_OutString(evt->chars);
            // UART0_OutString("\r\n");
            // if (temp != BLE_FRAME_START_TOKEN) {
            //     // print out data.

            // } else if (temp == BLE_FRAME_START_TOKEN) {
            //     UART0_OutString("start\r\n");
            //     status_ = Q_TRAN(&BLE_uart_packing);
            //     break;
            // }

            status_ = Q_HANDLED();
            break;
            }

        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }

    }
            return status_;
}
    static QState BLE_uart_packing(BLE_uart* const me, QEvt const* const e) {
        QState status_;
        switch (e->sig) {
            case Q_ENTRY_SIG: {
                status_ = Q_HANDLED();
                break;
            }

            case Q_EXIT_SIG: {
                status_ = Q_HANDLED();
                break;
            }
            // packing
            case NEW_UART_DATA_SIG: {
                char temp = Q_EVT_CAST(UartEvt)->temp;
                if (temp != BLE_FRAME_STOP_TOKEN) {
                    // pack data.
                    UART0_OutString("packing\r\n");
                    me->urtFrame[me->urtIdx] = temp;
                    me->urtIdx++;
                    // UART0_OutString("buff\r\n");
                    // UART0_OutString(me->urtFrame);
                    status_ = Q_HANDLED();
                    break;
                }
                if (temp == BLE_FRAME_STOP_TOKEN) {
                    UART0_OutString("end\r\n");
                    UART0_OutString(me->urtFrame);
                    me->urtIdx = 0;
                    me->urtFrameState = WAITING;
                    UART0_OutString("\r\n");
                }

                status_ = Q_TRAN(&BLE_uart_start);
                break;
            }
            default: {
                status_ = Q_SUPER(&BLE_uart_start);
                break;
            }
        }
        return status_;
    }
    static QState BLE_uart_off(BLE_uart* const me, QEvt const* const e) {
        QState status_;

        switch (e->sig) {  //*wait for event signal (5).
            case TIMEOUT1_SIG: {
                status_ = Q_TRAN(&BLE_uart_start);
                break;
            }

            default: {                               //* go off's super state on start (2)
                status_ = Q_SUPER(&BLE_uart_start);  // BUG: had this as (&QHsm_top)
                break;
            }
        }

        return status_;
    }

  

    
