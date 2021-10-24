/**
 * @file logger.h
 * @author Eddie Mhako (eddie.mhako@sky.com)
 * @brief 
 * @version 0.1
 * @date 2021-09-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "defs.h"

enum DPPSignals {
    EAT_SIG = Q_USER_SIG,  // published by Table to let a philosopher eat
    BUTTON1_PRESSED_SIG,
    BUTTON1_DEPRESSED_SIG,
    BUTTON2_PRESSED_SIG,
    BUTTON2_DEPRESSED_SIG,
    BUTTON3_PRESSED_SIG,
    BUTTON3_DEPRESSED_SIG,
    BUTTON4_PRESSED_SIG,
    BUTTON4_DEPRESSED_SIG,
    JOYSTICK_PRESSED_SIG,
    JOYSTICK_DEPRESSED_SIG,
    MAX_PUB_SIG,                 // the last published signal
    TRIGGER_TIMEOUT_SIG,         // Trigger adc data capture timeout signal
    NEW_SOUND_DATA_SIG,          // process new sound data buffer
    GET_SOUND_DATA_SIG,          // start sound data collection from idle state
    STOP_SOUND_DATA_SIG,         // stop sound data collection goto to idle state
    NEW_UART_DATA_SIG,
    SHOW_UART_DATA_SIG,
    DISPLAY_SOUND_MAG_SIG,       // sound data magnitude
    DISPLAY_UPDATE_TIMEOUT_SIG,  // update the display state
    PACKING_UART_DATA_SIG,
    POLLING_TIMEOUT_SIG,         // the periodic timeout signal
    DATA_READY_SIG,              // the invented reminder signal
    PROCESS_DATA_SIG,            // the invented reminder signal
    TERMINATE_SIG,               // terminate the application
    NEW_LIGHT_DATA_SIG,          // posted directlty to blinky for display, dynamic
    HEARTBEAT_TICK_SIG,
    HEARTBEAT_ON_SIG,            // heartbeat on signal
    HEARTBEAT_OFF_SIG,           // heartbeat off signal
    HEARTBEAT_SET_SIG,           // set the heartbeat rate based on system mode.
    HEARTBEAT_TIMEOUT_SIG,       // Heartbeat timeout signal
    TIMEOUT_SIG,                 // used by Philosophers for time events
    TIMEOUT1_SIG,                // used by Philosophers for time events
    MAX_SIG                      // the last signal
};

/* OPT3001Evt light sensor
 * ......................................................*/
typedef struct {
  /* protected: */
  QEvt super;

  /* public: */
  uint32_t lightData;
} OPT3001Evt;

/* Events DspSoundEvt========================================================= */
typedef struct {
  /* protected: */
  QEvt super;

  /* public: */
  int16_t soundMagBuff[SAMPLE_LENGTH/2];
  uint8_t buffSize;
} DspSoundEvt;

/* Events RawSoundEvt========================================================= */

typedef struct {
  /* protected: */
  QEvt super;

  /* public: */
  int16_t rawSoundBuf[SAMPLE_LENGTH];
  int32_t sound;
} RawSoundEvt;

/* Events HeartbeatEvt========================================================= */
enum SysMode
{
  MODE_1, // todo: what the modes does
};
typedef struct {
  /* protected: */
  QEvt super;

  /* public: */
} HeartbeatEvt;

/* Events UartEvt========================================================= */
enum {
  UART1_FIFO_LEN = 8
}; // only using 8bits of the uart register see@ UART Int.
typedef struct {
  /* protected: */
  QEvt super;

  /* public: */
  uint16_t temp;
  uint16_t temp2;
  uint16_t urtCharToInt;
  uint8_t len;                // length of string.
  char chars[UART1_FIFO_LEN]; // holds received string
} UartEvt;


void UI_ctor(void);

extern QActive *const AO_UI;



/* AO_BLE_uart  ##########################################################*/
void BLE_uart_ctor(void);

extern QActive *const AO_BLE_uart;

/* AO_BLE_uart  ##########################################################*/

/* AO_soundSensor  ##########################################################*/
void soundSensor_ctor(void);

extern QActive *const AO_soundSensor;

/* AO_soundSensor  ##########################################################*/

/* AO_Sensor  ##########################################################*/
void Sensor_ctor(void);

extern QActive *const AO_Sensor;

/* AO_Sensor  ##########################################################*/



#ifdef qxk_h
    void
    Test1_ctor(void);
    extern QXThread * const XT_Test1;
    void Test2_ctor(void);
    extern QXThread * const XT_Test2;
#endif /* qxk_h */

#endif /* LOGGER_H */

    /**
     * @brief 1
     * The constant MAX_PUB_SIG delimits the published signals from the rest.
     * The publish-subscribe event delivery mechanism consumes some RAM, which
     * is proportional to the number of published signals. I save some RAM by
     * providing the lower limit of published signals to QP (MAX_PUB_SIG) rather
     * than the maximum of all signals used in the application.
     */
