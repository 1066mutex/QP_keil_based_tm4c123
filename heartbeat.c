
#include "qpc.h"
#include "bsp.h"
#include "bsp_bp.h"
#include "logger.h"
#include "heartbeat.h"

/* Heartbeat component --------------------*/

void Heartbeat_ctor(Heartbeat *const me) {
    QHsm_ctor(&me->super, Q_STATE_CAST(&Heartbeat_initial));
}

/* Heartbeat SM .................................................*/
QState Heartbeat_initial(Heartbeat *const me, QEvt const *const e) {
    
    me->Heartbeat_time = 12U * 60U;
    (void)e; // avoid compiler warning about unused parameter
    BSP_ledRedOff();
    return Q_TRAN(&Heartbeat_off);
}

/* Component Heartbeat SM off ............................................*/
QState Heartbeat_off(Heartbeat *const me, QEvt const *const e) {
    QState status_;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            /* while in the off state, the Heartbeat is kept off */
            BSP_ledRedOff();
            status_ = Q_HANDLED();
            break;
        }
        
        case Q_EXIT_SIG: {
            /* upon exit, the Heartbeat is set */
            status_ = Q_HANDLED();
            break;
        }

        case HEARTBEAT_TICK_SIG: {
            status_ = Q_TRAN(&Heartbeat_on);
            break;
        }

        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status_;
}
/* Components Heartbeat SM on} .............................................*/
QState Heartbeat_on(Heartbeat *const me, QEvt const *const e) {
    QState status_;
    switch (e->sig) {
        
        case Q_ENTRY_SIG: {
        /* on entry the Heartbeat set on */
        BSP_ledRedOn();
        status_ = Q_HANDLED();
        break;
        }
        case HEARTBEAT_TICK_SIG: {
            status_ = Q_TRAN(&Heartbeat_off);
            break;
        }
        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status_;
}
/*$enddef${Components Heartbeat ----------------------------------------------*/