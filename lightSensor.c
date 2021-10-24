
#include "qpc.h"
#include "bsp.h"
#include "logger.h"
#include "bsp_bp.h"
#include "profile.h" // TODO: not sure if this is necessary remove and test build

/* Active object class  ############################################*/

/*..........................................................................*/
typedef struct SensorTag {    /* the Sensor active object */
    QActive super;            /* inherit QActive */

    QTimeEvt timeEvt;         /* private time event generator */
    uint16_t lightSensorPollingRate;         /* private light sensor polling rate */
    uint32_t lightData;         /* private light sensor measurements */
  
} Sensor;


void Sensor_ctor(void);

/* hierarchical state machine ... */
static QState Sensor_initial   (Sensor * const me, QEvt const * const e);
static QState Sensor_polling   (Sensor * const me, QEvt const * const e);
static QState Sensor_processing(Sensor * const me, QEvt const * const e);
static QState Sensor_idle      (Sensor * const me, QEvt const * const e);
static QState Sensor_busy      (Sensor * const me, QEvt const * const e);

/* Local objects -----------------------------------------------------------*/
static Sensor l_Sensor; /* the single instance of the Sensor active object */

/* Global-scope objects ----------------------------------------------------*/
QActive *const AO_Sensor = &l_Sensor.super; /* "opaque" AO pointer */

/*..........................................................................*/
void Sensor_ctor(void) {
    Sensor *me = &l_Sensor;
    QActive_ctor(&me->super, (QStateHandler)&Sensor_initial);
    QTimeEvt_ctorX(&me->timeEvt, &me->super, POLLING_TIMEOUT_SIG, 0U);
}

/* HSM definition ----------------------------------------------------------*/
QState Sensor_initial(Sensor * const me, QEvt const * const e) {
    (void)e; /* unused parameter */
//! trigger sensor here or in the init_sig ???
    BSP_BP_LightSensor_Init();  //TODO: move to bsp_init().
    me->lightSensorPollingRate = 95U; // 

    return Q_TRAN(&Sensor_polling);
}

/*..........................................................................*/
QState Sensor_polling(Sensor * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            /* periodic timeout in 1/2 second and every 1/2 second */
            /* periodic timeout in 800ms BSP_TICKS_PER_SEC x 0.8 */
            QTimeEvt_armX(&me->timeEvt, me->lightSensorPollingRate, me->lightSensorPollingRate);
            // start light measurement.
            
            BSP_BP_LightSensor_Start(); 
            status = Q_HANDLED();
            break;
        }
        case Q_INIT_SIG: {
            //! can possibly trigger sensor here???
            status = Q_TRAN(&Sensor_processing);
            break;
        }
        case POLLING_TIMEOUT_SIG: {
            /* NOTE: this constant event is statically pre-allocated.
            * It can be posted/published as any other event.
            */
            static const QEvt reminderEvt = { DATA_READY_SIG, 0U, 0U };
            // check if the sensor has new data ready.
            Profile_Toggle0();
            if ((BSP_BP_LightSensor_End(&me->lightData)) == 1U) { /* new data available? */
                Profile_Toggle4();
                QACTIVE_POST(&me->super, &reminderEvt, me);
                BSP_BP_LightSensor_Start();
            }
            status = Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status;
}
/*..........................................................................*/
QState Sensor_processing(Sensor * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_INIT_SIG: {
            status = Q_TRAN(&Sensor_idle);
            break;
        }
        default: {
            status = Q_SUPER(&Sensor_polling);
            break;
        }
    }
    return status;
}
/*..........................................................................*/
QState Sensor_idle(Sensor * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
           
        
            status = Q_HANDLED();
            break;
        }
        case DATA_READY_SIG: {
            static const QEvt porcessEvt = { PROCESS_DATA_SIG, 0U, 0U };
                QACTIVE_POST(&me->super, &porcessEvt, me);
            status = Q_TRAN(&Sensor_busy);
            break;
        }
        default: {
            status = Q_SUPER(&Sensor_processing);
            break;
        }
    }
    return status;
}
/*..........................................................................*/
QState Sensor_busy(Sensor * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            //! (op 1) disarm timeout*
            status = Q_HANDLED();
            break;
        }
        //! (op 1) exit action to start timer and trigger sensor.*

        //! ignore timeout if still processing data.
        case POLLING_TIMEOUT_SIG: {
            status = Q_HANDLED();     
            break;
        }
        //! need a signal to trigger data processing.
        case PROCESS_DATA_SIG: {
        // send processed data to blinky to be displayed.
        OPT3001Evt *ltData = Q_NEW(OPT3001Evt, NEW_LIGHT_DATA_SIG);
        ltData->lightData = me->lightData/100;
        QACTIVE_POST(AO_UI, &ltData->super, me);
        status = Q_TRAN(&Sensor_idle);
        break;
        }
        
        default: {
            status = Q_SUPER(&Sensor_processing);
            break;
        }
    }
    return status;
}

