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
#include "dpp.h"
#include "bsp.h"
#include "bsp_bp.h"

/* Active object class  ############################################*/

// AO_Heartbeat=======

typedef struct
{    // protected
    QActive super;
    // private
QTimeEvt timeEvt;
uint16_t heartbeat_time;
uint16_t buzzer_freq;
uint16_t joystickAxis0;
uint16_t joystickAxis1;
uint16_t accelaAxisX;
uint16_t accelaAxisY;
uint16_t accelaAxisZ;
uint16_t sound;
uint8_t tempAvg[2];
uint8_t tempAvgInx;
uint16_t urtCharToInt;
uint16_t ledPwm;
uint16_t urtIdx;
char urtFrame[20];
enum { WAITING = 1,
     GOT_START
      }urtFrameState;
    
}Heartbeat;

/* protected: */
static QState Heartbeat_initial(Heartbeat* const me, QEvt const* const e);
static QState Heartbeat_on     (Heartbeat* const me, QEvt const* const e);
static QState Heartbeat_start  (Heartbeat* const me, QEvt const* const e);
static QState Heartbeat_stop   (Heartbeat* const me, QEvt const* const e);
static QState Heartbeat_off    (Heartbeat* const me, QEvt const* const e);
//============================================================

// Local instance of Heartbeat object===============================
//
static Heartbeat l_Heartbeat;
/* Global-scope objects ----------------------------------------------------*/
QActive* const AO_Heartbeat = &l_Heartbeat.super;

// AO_Heartbeat constructor==================================
//
void Heartbeat_ctor(void){
    Heartbeat* me = (Heartbeat*)AO_Heartbeat;
    // add AO to qp
    // 
    QActive_ctor(&me->super, Q_STATE_CAST(&Heartbeat_initial));
    QTimeEvt_ctorX(&me->timeEvt, &me->super, TIMEOUT1_SIG, 0U);
}

static QState Heartbeat_initial(Heartbeat* const me, QEvt const* const e){
    (void)e;
    // QS_OBJ_DICTIONARY(&l_Heartbeat);
    // QS_FUN_DICTIONARY(&QHsm_top);
    // QS_FUN_DICTIONARY(&Heartbeat_initial); 
    // QS_FUN_DICTIONARY(&Heartbeat_off);
    // QS_FUN_DICTIONARY(&Heartbeat_on);
    // Q spy dummy code. 
    QS_SIG_DICTIONARY(BUTTON3_DEPRESSED_SIG, (void*)0);
    QS_SIG_DICTIONARY(BUTTON3_PRESSED_SIG, (void*)0);
    QS_SIG_DICTIONARY(BUTTON4_DEPRESSED_SIG, (void*)0);
    QS_SIG_DICTIONARY(BUTTON4_PRESSED_SIG, (void*)0);
    QS_SIG_DICTIONARY(NEW_TEMP_DATA_SIG, (void*)0);

    QActive_subscribe(&me->super, BUTTON4_DEPRESSED_SIG);
    QActive_subscribe(&me->super, BUTTON4_PRESSED_SIG);
    QActive_subscribe(&me->super, JOYSTICK_PRESSED_SIG);
    QActive_subscribe(&me->super, BUTTON3_PRESSED_SIG);

    BSP_ledRedOff();
   // BSP_LCD_Init(); // TODO: move to bsp_init()
   // BSP_BP_Joystick_Init();  // TODO: move to bsp_init()
    // TODO: make display objects with parameters for strings, location, colour etc.
    //       don't have to use magic numbers.
    BSP_LCD_DrawString(0, 4, "SysTemp:", LCD_YELLOW);
    BSP_LCD_DrawString(0, 5, "Acc x:", LCD_ORANGE);
    BSP_LCD_DrawString(0, 6, "Acc y:", LCD_ORANGE);
    BSP_LCD_DrawString(0, 7, "Acc z:", LCD_ORANGE);
    BSP_LCD_DrawString(0, 8, "light:", LCD_YELLOW);
    me->buzzer_freq = 10;
    me->urtIdx = 0 ;
    me->urtFrameState = WAITING;    // uart frame state machine start state.
    return Q_TRAN(&Heartbeat_off);  //*go to off (1)
}
static QState Heartbeat_start(Heartbeat* const me, QEvt const* const e){

    // super state for on and off states
    
    QState status_;
    switch (e->sig) {

        case Q_ENTRY_SIG: {
            /* arm the time event to expire in half a second and every half second */
            QTimeEvt_armX(&me->timeEvt, BSP_TICKS_PER_SEC / 2U, BSP_TICKS_PER_SEC / 16U);
            //! the timer will expire once for the first value, then use the second value
            //! continuously
           //BSP_Microphone_Get();
            status_ = Q_HANDLED();  //*do entry actions (3).
            break;
        }
        
        case Q_EXIT_SIG: {
            // dis-arm timer
            QTimeEvt_disarm(&me->timeEvt);
            //BSP_BP_LedBlueOff();
            BSP_BP_LedBlueDuty(0);
            status_ = Q_HANDLED();
            break;
        }
        case NEW_LIGHT_DATA_SIG: {
            BSP_LCD_SetCursor(8, 8);
            BSP_LCD_OutUDec5(Q_EVT_CAST(OPT3001Evt)->lightData, LCD_CYAN);
            //! output to ble
           // UART1_OutUDec(Q_EVT_CAST(OPT3001Evt)->lightData);
            //UART1_OutString("\r\n");
            status_ = Q_HANDLED();
            break;
        }
        case BUTTON3_PRESSED_SIG: {
            UART0_OutString("BTN3");

            me->urtIdx = 0;
            BSP_LCD_DrawString(9, 10, "BTN3", LCD_CYAN);
            //memset(me->urtFrame,1,strlen(me->urtFrame));
            status_ = Q_HANDLED();
            break;
        }
        case BUTTON4_PRESSED_SIG: {
            UART0_OutString("BTN4");
            status_ = Q_HANDLED();
            break;
        }
        case Q_INIT_SIG: {
            
            status_ = Q_TRAN(&Heartbeat_off);  //*go back to off (4).
            break;
        }
        case JOYSTICK_PRESSED_SIG: {
            BSP_BP_Buzzer_Freq(0.00033);
            status_ = Q_TRAN(&Heartbeat_stop);
            break;
        }
        case NEW_TEMP_DATA_SIG: {
            // average
            // if (me->tempAvgInx < 2){
            //     me->tempAvg[me->tempAvgInx] = Q_EVT_CAST(TempEvt)->temp;
            //     me->tempAvgInx++;
            // }else{

            // BSP_LCD_SetCursor(9, 4);
            // BSP_LCD_OutUDec((me->tempAvg[0] + me->tempAvg[1])/2, LCD_YELLOW);
            // me->tempAvgInx = 0;

            // }
            //BSP_Microphone_Input(&me->sound);
            BSP_LCD_SetCursor(3, 3);
            BSP_LCD_OutUDec4(Q_EVT_CAST(SoundEvt)->soundBuf[0], LCD_GREEN);  // mic data

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
static QState Heartbeat_on(Heartbeat* const me, QEvt const* const e){
    QState status_;
    switch (e->sig) {
        
        case Q_ENTRY_SIG: {
            //BSP_BP_LedBlueOn();
            BSP_BP_LedBlueDuty(20);
            //BSP_BP_Buzzer_Set(me->buzzer_freq);
            //BSP_Microphone_Get(); //! (1)trigger microphone ADC convertion
            BSP_Joystick_Trigger();
            BSP_AccelerometerGet();
            //! print to uart device
            
            status_ = Q_HANDLED();
            break;
        }
        
        case Q_EXIT_SIG: {
            if (me->buzzer_freq > 2000) {
                // reset frequency
                me->buzzer_freq = 100;
            } else {

                me->buzzer_freq += 10;
            }
            //BSP_BP_LedBlueOff();
            //BSP_BP_Buzzer_Set(0);
            BSP_Microphone_Get();  //! (2)trigger microphone ADC convertion
            BSP_BP_LedBlueDuty(5);
            BSP_Joystick_Input(&me->joystickAxis0, &me->joystickAxis1);
           // BSP_LCD_FillRect(2, 100, 120, 12, LCD_BLUE);
            BSP_LCD_SetCursor(3, 1);
            BSP_LCD_OutUDec4(me->joystickAxis0, LCD_RED);
            BSP_LCD_SetCursor(3, 2);
            BSP_LCD_OutUDec4(me->joystickAxis1, LCD_BLUE);

            BSP_Accelerometer_Input(&me->accelaAxisX, &me->accelaAxisY, &me->accelaAxisZ);
            BSP_LCD_SetCursor(6, 5);
            BSP_LCD_OutUDec4(me->accelaAxisX, LCD_RED);
            BSP_LCD_SetCursor(6, 6);
            BSP_LCD_OutUDec4(me->accelaAxisY, LCD_BLUE);
            BSP_LCD_SetCursor(6, 7);
            BSP_LCD_OutUDec4(me->accelaAxisZ, LCD_BLUE);
           // BSP_SystemTempGet(); // trigger temp Reading.
            //BSP_Microphone_Get();  //! this is blocking, waits for the conversion to complete.
            status_ = Q_HANDLED();
            break;
        }
        
        case TIMEOUT1_SIG: {
            status_ = Q_TRAN(&Heartbeat_off);
            break;
        }
        default: {
            status_ = Q_SUPER(&Heartbeat_start);
            break;
        }
    }
    return status_;
}
static QState Heartbeat_off(Heartbeat* const me, QEvt const* const e){
    QState status_;

    switch (e->sig) { //*wait for event signal (5).
        case TIMEOUT1_SIG: {
            
            status_ = Q_TRAN(&Heartbeat_on);
            break;
        }
        
        default: { //* go off's super state on start (2)
            status_ = Q_SUPER(&Heartbeat_start);  // BUG: had this as (&QHsm_top)
            break;
        }
    }

    return status_;
}

static QState Heartbeat_stop(Heartbeat* const me, QEvt const* const e){
    QState status_;

    switch (e->sig) {
        case JOYSTICK_DEPRESSED_SIG: {
            BSP_BP_Buzzer_Freq(0.00);
            //BSP_LCD_DrawString(0, 5, "Hello www", LCD_YELLOW);
            
            status_ = Q_TRAN(&Heartbeat_start);
            break;
        }
        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status_;
}

// TODO: handle the NEW_LIGHT_DATA_SIG signal with the



