/**
 * @file ui.c
 * @author Eddie Mhako (eddie.mhako@sky.com)
 * @brief  User interface active object. 
 *         Manages all user interface output display, leds and buzzer. 
 * @version 0.1
 * @date 2021-09-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "qpc.h"
#include "bsp.h"
#include "bsp_bp.h"
#include "logger.h"
#include "heartbeat.h"

Q_DEFINE_THIS_FILE

/* Active object class  ############################################*/

// AO_UI=======

typedef struct
{  // protected
    QActive super;
    
    /* private: */
    Heartbeat heartbeat;                 // private heartbeat component SM
    QTimeEvt updateDisplayTimeEvt;
    QTimeEvt heartbeatTimeEvt;
    uint16_t heartbeat_time;
    uint16_t buzzerFreq;  
    int32_t sound;
    uint8_t tempAvg[2];
    uint8_t tempAvgInx;
    uint16_t ledPwm;

} UI;

/* protected: */
static QState UI_initial(UI* const me, QEvt const* const e);
static QState UI_Active(UI* const me, QEvt const* const e);

//============================================================

// Local instance of UI object===============================
//
static UI l_UI;
/* Global-scope objects ----------------------------------------------------*/
QActive* const AO_UI = &l_UI.super;

/* AO_UI constructor --------------------------------------------------------*/

void UI_ctor(void) {
    UI* me = (UI*)AO_UI;
   
    /* add AO and component SM to qp */
    QActive_ctor(&me->super, Q_STATE_CAST(&UI_initial));
    
    /* orthogonal component ctor */
    Heartbeat_ctor(&me->heartbeat);  // this links the component to the UI active object

    /* private time event ctor */
    QTimeEvt_ctorX(&me->updateDisplayTimeEvt, &me->super, DISPLAY_UPDATE_TIMEOUT_SIG, 0U);
    /* heartbeat time event ctor */
    QTimeEvt_ctorX(&me->heartbeatTimeEvt, &me->super, HEARTBEAT_TIMEOUT_SIG, 0U);
}

static QState UI_initial(UI* const me, QEvt const* const e) {
    (void)e;
    // TODO: delete if not using Q spy
    // QS_OBJ_DICTIONARY(&l_UI);
    // QS_FUN_DICTIONARY(&QHsm_top);
    // QS_FUN_DICTIONARY(&UI_initial);
    // QS_FUN_DICTIONARY(&UI_off);
    // QS_FUN_DICTIONARY(&UI_on);
    // Q spy dummy code.
    QS_SIG_DICTIONARY(BUTTON3_DEPRESSED_SIG, (void*)0);
    QS_SIG_DICTIONARY(BUTTON3_PRESSED_SIG, (void*)0);
    QS_SIG_DICTIONARY(BUTTON4_DEPRESSED_SIG, (void*)0);
    QS_SIG_DICTIONARY(BUTTON4_PRESSED_SIG, (void*)0);
   

    QActive_subscribe(&me->super, BUTTON4_DEPRESSED_SIG);
    QActive_subscribe(&me->super, BUTTON4_PRESSED_SIG);
    QActive_subscribe(&me->super, JOYSTICK_PRESSED_SIG);
    QActive_subscribe(&me->super, JOYSTICK_DEPRESSED_SIG);
    QActive_subscribe(&me->super, BUTTON3_PRESSED_SIG);


    // TODO: make display objects with parameters for strings, location, colour etc.
    //       don't have to use magic numbers.
    BSP_LCD_DrawString(0, 1, "SYSTEM ACTIVE", LCD_YELLOW);
   
    
    /* arm the time event to expire in half a second and every 25ms */
    QTimeEvt_armX(&me->updateDisplayTimeEvt, BSP_TICKS_PER_SEC / 2U, BSP_TICKS_PER_SEC / 40U);

    /* arm the time event to expire in half a second and every half a second after*/
    QTimeEvt_armX(&me->heartbeatTimeEvt, BSP_TICKS_PER_SEC / 2U, BSP_TICKS_PER_SEC / 2U);

    /* (!) trigger the initial transition in the heartbeat component  */
    QHSM_INIT((QHsm*)&me->heartbeat, (QEvt*)0);
    
    return Q_TRAN(&UI_Active);  //*go to off (1)
}

// main superstate
static QState UI_Active(UI* const me, QEvt const* const e) {
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

        case DISPLAY_UPDATE_TIMEOUT_SIG: {
            // call display update function
            BSP_LCD_DrawString(0, 1, "READY", LCD_YELLOW);
            status_ = Q_HANDLED();
            break;
        }
        case HEARTBEAT_TIMEOUT_SIG: {
            /* asynchronously post the event to the container AO */
            HeartbeatEvt pe; /* temporary synchronous event for the component */
            pe.super.sig = HEARTBEAT_TICK_SIG;
            /* (!) synchronously dispatch to the orthogonal component */
            QHSM_DISPATCH(&me->heartbeat.super, &pe.super);
            status_ = Q_HANDLED();
            break;
        }

        case HEARTBEAT_SET_SIG: {
            /* set the Heartbeat time based on the system mode */
            
            status_ = Q_HANDLED();
            break;
        }

        default: {                                //* go off's super state on start (2)
            status_ = Q_SUPER(&QHsm_top);  
            break;
        }
    }

    return status_;
}

// update the display at a fixed frequency or when we receive an event.
// use draw fast line to draw graph.
// maintain UI
// buzzer output when buttons are pressed or selecting a display option. make a sound when pressed don't wait for release
// or keep making sound till release. each button can have a unique sound 
