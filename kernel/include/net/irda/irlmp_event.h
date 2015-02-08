

#ifndef IRLMP_EVENT_H
#define IRLMP_EVENT_H

/* A few forward declarations (to make compiler happy) */
struct irlmp_cb;
struct lsap_cb;
struct lap_cb;
struct discovery_t;

/* LAP states */
typedef enum {
	/* IrLAP connection control states */
	LAP_STANDBY,             /* No LAP connection */
	LAP_U_CONNECT,           /* Starting LAP connection */
	LAP_ACTIVE,              /* LAP connection is active */
} IRLMP_STATE;

/* LSAP connection control states */
typedef enum {
	LSAP_DISCONNECTED,        /* No LSAP connection */
	LSAP_CONNECT,             /* Connect indication from peer */
	LSAP_CONNECT_PEND,        /* Connect request from service user */
	LSAP_DATA_TRANSFER_READY, /* LSAP connection established */          
	LSAP_SETUP,               /* Trying to set up LSAP connection */
	LSAP_SETUP_PEND,          /* Request to start LAP connection */
} LSAP_STATE;

typedef enum {
	/* LSAP events */
 	LM_CONNECT_REQUEST,
 	LM_CONNECT_CONFIRM,
	LM_CONNECT_RESPONSE,
 	LM_CONNECT_INDICATION, 	
	
	LM_DISCONNECT_INDICATION,
	LM_DISCONNECT_REQUEST,

 	LM_DATA_REQUEST,
	LM_UDATA_REQUEST,
 	LM_DATA_INDICATION,
	LM_UDATA_INDICATION,

	LM_WATCHDOG_TIMEOUT,

	/* IrLAP events */
	LM_LAP_CONNECT_REQUEST,
 	LM_LAP_CONNECT_INDICATION, 
 	LM_LAP_CONNECT_CONFIRM,
 	LM_LAP_DISCONNECT_INDICATION, 
	LM_LAP_DISCONNECT_REQUEST,
	LM_LAP_DISCOVERY_REQUEST,
 	LM_LAP_DISCOVERY_CONFIRM,
	LM_LAP_IDLE_TIMEOUT,
} IRLMP_EVENT;

extern const char *const irlmp_state[];
extern const char *const irlsap_state[];

void irlmp_watchdog_timer_expired(void *data);
void irlmp_discovery_timer_expired(void *data);
void irlmp_idle_timer_expired(void *data);

void irlmp_do_lap_event(struct lap_cb *self, IRLMP_EVENT event, 
			struct sk_buff *skb);
int irlmp_do_lsap_event(struct lsap_cb *self, IRLMP_EVENT event, 
			struct sk_buff *skb);

#endif /* IRLMP_EVENT_H */




