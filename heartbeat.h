#ifndef HEARTBEAT_H
#define HEARTBEAT_H



/* Components::Heartbeat .....................................................*/
typedef struct {
  /* protected: */
  QHsm super;

  /* private: */
  uint32_t Heartbeat_time;
} Heartbeat;

/* public: */
void Heartbeat_ctor(Heartbeat *const me);

/* protected: */
QState Heartbeat_initial(Heartbeat *const me, QEvt const *const e);
QState Heartbeat_off(Heartbeat *const me, QEvt const *const e);
QState Heartbeat_on(Heartbeat *const me, QEvt const *const e);
/* enddecl Components Heartbeat
 * ------------------------------------------------*/

/* Components Heartbeat ctor
 * ---------------------------------------------------*/
void Heartbeat_ctor(Heartbeat *const me);
/* enddecl Components Heartbeat ctor ---------------------------------------*/

#endif /* HEARTBEAT_H */
