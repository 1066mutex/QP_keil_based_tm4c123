/*$file${.::dpp.h} #########################################################*/
/*
* Model: dpp.qm
* File:  ${.::dpp.h}
*
* This code has been generated by QM tool (https://state-machine.com/qm).
* DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*/
/*$endhead${.::dpp.h} ######################################################*/
#ifndef dpp_h
#define dpp_h

//TODO: ensure that all signals are kept in appropriate location. see @brief 1
enum DPPSignals {
  EAT_SIG = Q_USER_SIG, /* published by Table to let a philosopher eat */
  DONE_SIG,             /* published by Philosopher when done eating */
  PAUSE_SIG,            /* published by BSP to pause serving forks */
  SERVE_SIG,            /* published by BSP to serve re-start serving forks */
  TEST_SIG,             /* published by BSP to test the application */
  BUTTON3_PRESSED_SIG,
  BUTTON3_DEPRESSED_SIG,
  BUTTON4_PRESSED_SIG,
  BUTTON4_DEPRESSED_SIG,
  JOYSTICK_PRESSED_SIG,
  JOYSTICK_DEPRESSED_SIG,
  NEW_TEMP_DATA_SIG,
  MAX_PUB_SIG,            /* the last published signal see: @brief 1 */
  POLLING_TIMEOUT_SIG,    /* the periodic timeout signal */
  DATA_READY_SIG,         /* the invented reminder signal */
  PROCESS_DATA_SIG,       /* the invented reminder signal */
  TERMINATE_SIG,          /* terminate the application */
  NEW_LIGHT_DATA_SIG,     /* posted direclty to blinky for display, dynamic */
  HUNGRY_SIG,             /* posted direclty to Table from hungry Philo */
  TIMEOUT_SIG,            /* used by Philosophers for time events */
  TIMEOUT1_SIG,           /* used by Philosophers for time events */
  MAX_SIG                 /* the last signal */
};

/*$declare${Events::TableEvt} ##############################################*/
/*${Events::TableEvt} ......................................................*/
typedef struct {
/* protected: */
    QEvt super;

/* public: */
    uint8_t philoNum;
} TableEvt;
/*$enddecl${Events::TableEvt} ##############################################*/

/* OPT3001Evt light sensor ......................................................*/
typedef struct {
/* protected: */
    QEvt super;

/* public: */
    uint32_t lightData;
} OPT3001Evt;

/* Events TempEvt========================================================= */

typedef struct{
/* protected: */
    QEvt super;

/* public: */
    uint32_t temp;
} TempEvt;

/* number of philosophers */
#define N_PHILO ((uint8_t)5)

/*$declare${AOs::Philo_ctor} ###############################################*/
/*${AOs::Philo_ctor} .......................................................*/
void Philo_ctor(void);
/*$enddecl${AOs::Philo_ctor} ###############################################*/
/*$declare${AOs::AO_Philo[N_PHILO]} ########################################*/
extern QMActive * const AO_Philo[N_PHILO];
/*$enddecl${AOs::AO_Philo[N_PHILO]} ########################################*/

/*$declare${AOs::Table_ctor} ###############################################*/
/*${AOs::Table_ctor} .......................................................*/
void Table_ctor(void);
/*$enddecl${AOs::Table_ctor} ###############################################*/
/*$declare${AOs::AO_Table} #################################################*/
extern QActive * const AO_Table;
/*$enddecl${AOs::AO_Table} #################################################*/

/* AO_Heartbeat  ##########################################################*/
void Heartbeat_ctor(void);

extern QActive * const AO_Heartbeat;

/* AO_Heartbeat  ##########################################################*/

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

#endif /* dpp_h */

    /**
     * @brief 1
     * The constant MAX_PUB_SIG delimits the published signals from the rest.
     * The publish-subscribe event delivery mechanism consumes some RAM, which is
     * proportional to the number of published signals. I save some RAM by
     * providing the lower limit of published signals to QP (MAX_PUB_SIG) rather
     * than the maximum of all signals used in the application.
     */
