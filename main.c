/*****************************************************************************
* Product: DPP example, FreeRTOS kernel
* Last updated for version 6.4.0
* Last updated on  2019-02-08
*
*                    Q u a n t u m  L e a P s
*                    ------------------------
*                    Modern Embedded Software
*
* Copyright (C) 2005-2019 Quantum Leaps, LLC. All rights reserved.
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
* https://www.state-machine.com
* mailto:info@state-machine.com
*****************************************************************************/
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"


Q_DEFINE_THIS_FILE

/*..........................................................................*/
int main() {
    static QEvt const *tableQueueSto[N_PHILO];
    static QEvt const *philoQueueSto[N_PHILO][N_PHILO];
    static QEvt const *heartbeatQueueSto[10]; //! heartbeat queue.
    static QEvt const *sensorQueueSto[10]; //! sensor queue.
    static QEvt const *BLE_uartQueueSto[10];  //! sensor queue.
    static QSubscrList subscrSto[MAX_PUB_SIG];
    static QF_MPOOL_EL(TableEvt) smlPoolSto[2*N_PHILO]; /* small pool */
    static QF_MPOOL_EL(UartEvt) medlPoolSto[2 * N_PHILO]; /* medium pool */

    static StackType_t philoStack[N_PHILO][configMINIMAL_STACK_SIZE];
    static StackType_t tableStack[configMINIMAL_STACK_SIZE];
    static StackType_t heartbeatStack[configMINIMAL_STACK_SIZE];//! heartbeat stack
    static StackType_t sensorStack[configMINIMAL_STACK_SIZE];//! sensor stack
    static StackType_t BLE_uartStack[configMINIMAL_STACK_SIZE];  //! BLE_uart stack

    uint8_t n;

    Philo_ctor(); /* instantiate all Philosopher active objects */
    Table_ctor(); /* instantiate the Table active object */
    Heartbeat_ctor();
    Sensor_ctor();
    BLE_uart_ctor();
    QF_init(); /* initialize the framework and the underlying RT kernel */

    /* initialize publish-subscribe... */
    QF_psInit(subscrSto, Q_DIM(subscrSto));

    /* initialize event pools... */
    //!  for more info see book page 350
    QF_poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));
    QF_poolInit(medlPoolSto, sizeof(medlPoolSto), sizeof(medlPoolSto[0]));

    /* initialize the Board Support Package
    * NOTE: BSP_init() is called *after* initializing publish-subscribe and
    * event pools, to make the system ready to accept SysTick interrupts.
    * Unfortunately, the STM32Cube code that must be called from BSP,
    * configures and starts SysTick.
    */
    BSP_init();

    /* start the active objects... */
    for (n = 0U; n < N_PHILO; ++n) {
        QActive_setAttr(AO_Philo[n], TASK_NAME_ATTR, "Philo");
        QACTIVE_START(AO_Philo[n],   /* AO to start */
            (uint_fast8_t)(n + 1),   /* QP priority of the AO */
            philoQueueSto[n],        /* event queue storage */
            Q_DIM(philoQueueSto[n]), /* queue length [events] */
            philoStack[n],           /* stack storage */
            sizeof(philoStack[n]),   /* stack size [bytes] */
            (QEvt *)0);              /* initialization event (not used) */
    }

    QActive_setAttr(AO_Table, TASK_NAME_ATTR, "Table");
    QACTIVE_START(AO_Table,          /* AO to start */
        (uint_fast8_t)(N_PHILO + 3), /* QP priority of the AO */
        tableQueueSto,               /* event queue storage */
        Q_DIM(tableQueueSto),        /* queue length [events] */
        tableStack,                  /* stack storage */
        sizeof(tableStack),          /* stack size [bytes] */
        (QEvt *)0);                  /* initialization event (not used) */

    QActive_setAttr(AO_Heartbeat, TASK_NAME_ATTR, "Heartbeat");
    QACTIVE_START(AO_Heartbeat,          /* AO to start */
        (uint_fast8_t)(N_PHILO + 2), /* QP priority of the AO */
        heartbeatQueueSto,               /* event queue storage */
        Q_DIM(heartbeatQueueSto),        /* queue length [events] */
        heartbeatStack,                  /* stack storage */
        sizeof(heartbeatStack),          /* stack size [bytes] */
        (QEvt *)0);                  /* initialization event (not used) */

    QActive_setAttr(AO_Sensor, TASK_NAME_ATTR, "AO_Sensor");
    QACTIVE_START(AO_Sensor,                   /* AO to start */
                  (uint_fast8_t)(N_PHILO + 4), /* QP priority of the AO */
                  sensorQueueSto,           /* event queue storage */
                  Q_DIM(sensorQueueSto),    /* queue length [events] */
                  sensorStack,              /* stack storage */
                  sizeof(sensorStack),      /* stack size [bytes] */
                  (QEvt *)0);                  /* initialization event (not used) */

    QActive_setAttr(AO_BLE_uart, TASK_NAME_ATTR, "AO_BLE_uart");
    QACTIVE_START(AO_BLE_uart,                 /* AO to start */
                  (uint_fast8_t)(N_PHILO + 1), /* QP priority of the AO */
                  BLE_uartQueueSto,            /* event queue storage */
                  Q_DIM(BLE_uartQueueSto),     /* queue length [events] */
                  BLE_uartStack,               /* stack storage */
                  sizeof(BLE_uartStack),       /* stack size [bytes] */
                  (QEvt *)0);                  /* initialization event (not used) */

    return QF_run(); /* run the QF application */
}

