
#include "qpc.h"
#include "logger.h"
#include "bsp.h"

Q_DEFINE_THIS_FILE

/*..........................................................................*/
int main() {
    
    static QEvt const *UIQueueSto[10]; //!  UI queue.
    static QEvt const *sensorQueueSto[10];    //! sensor queue.
    static QEvt const *BLE_uartQueueSto[10];  //! sensor queue.
    static QEvt const *soundSensorQueueSto[10];  //! sensor queue.

    static QSubscrList subscrSto[MAX_PUB_SIG];
    // static QF_MPOOL_EL(TableEvt) smlPoolSto[2*N_PHILO]; /* small pool */
   // TODO: workout the memory pool requirements, this is temporarily
    static QF_MPOOL_EL(UartEvt) medlPoolSto[2]; // medium pool
    static QF_MPOOL_EL(RawSoundEvt) medlPoolSto1[5];  // medium pool

    static StackType_t UIStack[configMINIMAL_STACK_SIZE];//!  UI stack
    static StackType_t sensorStack[configMINIMAL_STACK_SIZE];   //! sensor stack
    static StackType_t BLE_uartStack[configMINIMAL_STACK_SIZE];  //! BLE_uart stack
    static StackType_t soundSensorStack[configMINIMAL_STACK_SIZE];  //! soundSensor stack

    /* instantiate all the active objects */
    UI_ctor();
    Sensor_ctor();
    BLE_uart_ctor();
    soundSensor_ctor();
    
    /* initialise the framework and the underlying RT kernel */
    QF_init(); 

    /* initialize publish-subscribe... */
    QF_psInit(subscrSto, Q_DIM(subscrSto));

    /* initialize event pools... */
    QF_poolInit(medlPoolSto, sizeof(medlPoolSto), sizeof(medlPoolSto[0]));
    QF_poolInit(medlPoolSto1, sizeof(medlPoolSto1), sizeof(medlPoolSto1[0]));
    

    /* initialize the Board Support Package
    * NOTE: BSP_init() is called *after* initializing publish-subscribe and
    * event pools, to make the system ready to accept SysTick interrupts.
    * Unfortunately, the STM32Cube code that must be called from BSP,
    * configures and starts SysTick.
    */
    BSP_init();


    QActive_setAttr(AO_UI, TASK_NAME_ATTR, " UI");
    QACTIVE_START(AO_UI,                        // AO to start 
                  (uint_fast8_t)(2),            // QP priority of the AO 
                   UIQueueSto,                  // event queue storage 
                  Q_DIM( UIQueueSto),           // queue length [events] 
                   UIStack,                     // stack storage 
                  sizeof( UIStack),             // stack size [bytes] 
                  (QEvt *)0);                   // initialization event (not used)

    QActive_setAttr(AO_soundSensor, TASK_NAME_ATTR, "AO_soundSensor");
    QACTIVE_START(AO_soundSensor,                // AO to start 
                  (uint_fast8_t)(3),             // QP priority of the AO 
                  soundSensorQueueSto,           // event queue storage 
                  Q_DIM(soundSensorQueueSto),    // queue length [events] 
                  soundSensorStack,              // stack storage 
                  sizeof(soundSensorStack),      // stack size [bytes] 
                  (QEvt *)0);                    // initialization event (not used) 

    QActive_setAttr(AO_Sensor, TASK_NAME_ATTR, "AO_Sensor");
    QACTIVE_START(AO_Sensor,                   // AO to start 
                  (uint_fast8_t)(5),           // QP priority of the AO 
                  sensorQueueSto,              // event queue storage 
                  Q_DIM(sensorQueueSto),       // queue length [events] 
                  sensorStack,                 // stack storage 
                  sizeof(sensorStack),         // stack size [bytes] 
                  (QEvt *)0);                  // initialization event (not used) 

    QActive_setAttr(AO_BLE_uart, TASK_NAME_ATTR, "AO_BLE_uart");
    QACTIVE_START(AO_BLE_uart,                 // AO to start 
                  (uint_fast8_t)(4),           // QP priority of the AO 
                  BLE_uartQueueSto,            // event queue storage 
                  Q_DIM(BLE_uartQueueSto),     // queue length [events] 
                  BLE_uartStack,               // stack storage 
                  sizeof(BLE_uartStack),       // stack size [bytes] 
                  (QEvt *)0);                  // Initialisation event (not used)

    return QF_run();                           // run the QF application 
}

