
#ifndef _android_modem_h_
#define _android_modem_h_

#include "sim_card.h"
#include "sms.h"

typedef struct AModemRec_*    AModem;

/* a function used by the modem to send unsolicited messages to the channel controller */
typedef void (*AModemUnsolFunc)( void*  opaque, const char*  message );

extern AModem      amodem_create( int  base_port, AModemUnsolFunc  unsol_func, void*  unsol_opaque );
extern void        amodem_set_legacy( AModem  modem );
extern void        amodem_destroy( AModem  modem );

/* send a command to the modem */
extern const char*  amodem_send( AModem  modem, const char*  cmd );

/* simulate the receipt on an incoming SMS message */
extern void         amodem_receive_sms( AModem  modem, SmsPDU  pdu );

typedef enum {
    A_RADIO_STATE_OFF = 0,          /* Radio explictly powered off (eg CFUN=0) */
    A_RADIO_STATE_ON,               /* Radio on */
    A_RADIO_SIM1_ONLY_STATE_ON,
    A_RADIO_SIM2__ONLY_STATE_ON
} ARadioState;

typedef enum {
    A_NETWORK_STATE_OFF = 0,          /* Radio explictly powered off (eg CFUN=0) */
    A_NETWORK_STATE_SIM1,               /* Radio on */
    A_NETWORK_STATE_SIM2
} ANetworkState;

extern ARadioState  amodem_get_radio_state( AModem modem );
extern void         amodem_set_radio_state( AModem modem, ARadioState  state );

extern ASimCard    amodem_get_sim( AModem  modem );


/* 'stat' for +CREG/+CGREG commands */
typedef enum {
    A_REGISTRATION_UNREGISTERED = 0,
    A_REGISTRATION_HOME = 1,
    A_REGISTRATION_SEARCHING,
    A_REGISTRATION_DENIED,
    A_REGISTRATION_UNKNOWN,
    A_REGISTRATION_ROAMING
} ARegistrationState;

typedef enum {
    A_GPRS_NETWORK_UNKNOWN = 0,
    A_GPRS_NETWORK_GPRS,
    A_GPRS_NETWORK_EDGE,
    A_GPRS_NETWORK_UMTS
} AGprsNetworkType;

extern ARegistrationState  amodem_get_voice_registration( AModem  modem );
extern void                amodem_set_voice_registration( AModem  modem, ARegistrationState    state );

extern ARegistrationState  amodem_get_data_registration( AModem  modem );
extern void                amodem_set_data_registration( AModem  modem, ARegistrationState    state );
extern void                amodem_set_data_network_type( AModem  modem, AGprsNetworkType   type );

extern AGprsNetworkType    android_parse_network_type( const char*  speed );
//YQ_TODO
extern int amodem_gemini_get_sims( AModem  modem);
extern int amodem_gemini_get_sms( AModem  modem);
extern void amodem_gemini_set_sims( AModem  modem,int sims);
extern void amodem_gemini_set_sms( AModem  modem,int asms);


typedef enum {
    A_NAME_LONG = 0,
    A_NAME_SHORT,
    A_NAME_NUMERIC,
    A_NAME_MAX  /* don't remove */
} ANameIndex;

/* retrieve operator name into user-provided buffer. returns number of writes written, including terminating zero */
extern int   amodem_get_operator_name ( AModem  modem, ANameIndex  index, char*  buffer, int  buffer_size );

/* reset one operator name from a user-provided buffer, set buffer_size to -1 for zero-terminated strings */
extern void  amodem_set_operator_name( AModem  modem, ANameIndex  index, const char*  buffer, int  buffer_size );


typedef enum {
    A_CALL_OUTBOUND = 0,
    A_CALL_INBOUND  = 1,
} ACallDir;

typedef enum {
    A_CALL_ACTIVE = 0,
    A_CALL_HELD,
    A_CALL_DIALING,
    A_CALL_ALERTING,
    A_CALL_INCOMING,
    A_CALL_WAITING
} ACallState;

typedef enum {
    A_CALL_VOICE = 0,
    A_CALL_DATA,
    A_CALL_FAX,
    A_CALL_UNKNOWN = 9
} ACallMode;

/*another is hold*/
typedef enum{
   ANOTHER_HOLD=0,
   ANOTHER_ACTIVE
}AAnotherHold;

#define  A_CALL_NUMBER_MAX_SIZE  16

typedef struct {
    int         id;
    ACallDir    dir;
    ACallState  state;
    AAnotherHold anotherstate;
    ACallMode   mode;
    int         multi;
    char        number[ A_CALL_NUMBER_MAX_SIZE+1 ];
} ACallRec, *ACall;

extern int    amodem_get_call_count( AModem  modem );
extern ACall  amodem_get_call( AModem  modem,  int  index );
extern ACall  amodem_find_call_by_number( AModem  modem, const char*  number );
extern int    amodem_add_inbound_call( AModem  modem, const char*  number );
extern int    amodem_update_call( AModem  modem, const char*  number, ACallState  state );
extern int    amodem_disconnect_call( AModem  modem, const char*  number );
extern int    amodem_accept_call(AModem modem, const char * fromNumber, ACallState state);
extern int    amodem_accepted_call(AModem modem, const char * fromNumber, ACallState state);
extern int    amodem_hold_call(AModem modem, const char * fromNumber, ACallState state);
extern int    amodem_held_call(AModem modem, const char * fromNumber);

extern int    amodem_cancel_call(AModem modem, const char * fromNumber);
extern int    amodem_canceled_call(AModem modem, const char * fromNumber);
extern int   amodem_reject_call(AModem modem, const char * fromNumber);
extern int   amodem_rejected_call(AModem modem, const char * fromNumber);
/**/

#endif /* _android_modem_h_ */
