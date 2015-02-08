#include "android/android.h"
#include "android_modem.h"
#include "android/utils/debug.h"
#include "android/utils/timezone.h"
#include "android/utils/system.h"
#include "sim_card.h"
#include "sysdeps.h"
#include <memory.h>
#include <stdarg.h>
#include <time.h>
#include <assert.h>
#include <stdio.h>
#include "sms.h"
#include "remote_call.h"


#define  DEBUG  1


#if  1
#  define  D_ACTIVE  VERBOSE_CHECK(modem)
#else
#  define  D_ACTIVE  DEBUG
#endif

#if 1
#  define  R_ACTIVE  VERBOSE_CHECK(radio)
#else
#  define  R_ACTIVE  DEBUG
#endif

#if DEBUG
#  define  D(...)   do { if (D_ACTIVE) fprintf( stderr, __VA_ARGS__ ); } while (0)
#  define  R(...)   do { if (R_ACTIVE) fprintf( stderr, __VA_ARGS__ ); } while (0)
#else
#  define  D(...)   ((void)0)
#  define  R(...)   ((void)0)
#endif

#define  CALL_DELAY_DIAL   1000
#define  CALL_DELAY_ALERT  1000
#define  REMOTE_CALL_SIM  10000
#define CHAR_A 'A'
#define CHAR_B 'B'
#define SIM1 1
#define SIM2 2
#define CHAR_NULL '\0';
#define  A_SIM_PIN_SIZE  8

#define  OPERATOR_HOME_INDEX 0
#define  OPERATOR_HOME_MCC   460
#define  OPERATOR_HOME_MNC   00
#define  OPERATOR_HOME_NAME  "Android"
#define  OPERATOR_HOME_MCCMNC  STRINGIFY(OPERATOR_HOME_MCC) \
                               STRINGIFY(OPERATOR_HOME_MNC)

#define  OPERATOR_ROAMING_INDEX 1
#define  OPERATOR_ROAMING_MCC   310
#define  OPERATOR_ROAMING_MNC   295
#define  OPERATOR_ROAMING_NAME  "TelKila"
#define  OPERATOR_ROAMING_MCCMNC  STRINGIFY(OPERATOR_ROAMING_MCC) \
                                  STRINGIFY(OPERATOR_ROAMING_MNC)
static const char*
handleHangup( const char*  cmd, AModem  modem );
#if DEBUG
static const char*  quote( const char*  line )
{
    static char  temp[1024];
    const char*  hexdigits = "0123456789abcdef";
    char*        p = temp;
    int          c;

    while ((c = *line++) != 0) {
        c &= 255;
        if (c >= 32 && c < 127) {
            *p++ = c;
        }
        else if (c == '\r') {
            memcpy( p, "<CR>", 4 );
            p += 4;
        }
        else if (c == '\n') {
            memcpy( p, "<LF>", 4 );strcat( p, "<LF>" );
            p += 4;
        }
        else {
            p[0] = '\\';
            p[1] = 'x';
            p[2] = hexdigits[ (c) >> 4 ];
            p[3] = hexdigits[ (c) & 15 ];
            p += 4;
        }
    }
    *p = 0;
    return temp;
}
#endif

extern AGprsNetworkType
android_parse_network_type( const char*  speed )
{
    const struct { const char* name; AGprsNetworkType  type; }  types[] = {
         { "gprs", A_GPRS_NETWORK_GPRS },
         { "edge", A_GPRS_NETWORK_EDGE },
         { "umts", A_GPRS_NETWORK_UMTS },
         { "hsdpa", A_GPRS_NETWORK_UMTS },  /* not handled yet by Android GSM framework */
         { "full", A_GPRS_NETWORK_UMTS },
         { NULL, 0 }
    };
    int  nn;

    for (nn = 0; types[nn].name; nn++) {
        if ( !strcmp(speed, types[nn].name) )
            return types[nn].type;
    }
    /* not found, be conservative */
    return A_GPRS_NETWORK_GPRS;
}

/* 'mode' for +CREG/+CGREG commands */
typedef enum {
    A_REGISTRATION_UNSOL_DISABLED     = 0,
    A_REGISTRATION_UNSOL_ENABLED      = 1,
    A_REGISTRATION_UNSOL_ENABLED_FULL = 2
} ARegistrationUnsolMode;

/* Operator selection mode, see +COPS commands */
typedef enum {
    A_SELECTION_AUTOMATIC,
    A_SELECTION_MANUAL,
    A_SELECTION_DEREGISTRATION,
    A_SELECTION_SET_FORMAT,
    A_SELECTION_MANUAL_AUTOMATIC
} AOperatorSelection;

/* Operator status, see +COPS commands */
typedef enum {
    A_STATUS_UNKNOWN = 0,
    A_STATUS_AVAILABLE,
    A_STATUS_CURRENT,
    A_STATUS_DENIED
} AOperatorStatus;

typedef struct {
    AOperatorStatus  status;
    char             name[3][16];
} AOperatorRec, *AOperator;

typedef struct AVoiceCallRec {
    ACallRec    call;
    SysTimer    timer;
    AModem      modem;
    char        is_remote;
    int from_sim;
} AVoiceCallRec, *AVoiceCall;

#define  MAX_OPERATORS  4

typedef enum {
    A_DATA_IP = 0,
    A_DATA_PPP
} ADataType;

#define  A_DATA_APN_SIZE  32

typedef struct {
    int        id;
    int        active;
    ADataType  type;
    char       apn[ A_DATA_APN_SIZE ];

} ADataContextRec, *ADataContext;

/* the spec says that there can only be a max of 4 contexts */
#define  MAX_DATA_CONTEXTS  4
#define  MAX_CALLS          4

#define  A_MODEM_SELF_SIZE   3

typedef struct AModemRec_
{
    /* Legacy support */
    char          supportsNetworkDataType;

    /* Radio state */
    ARadioState   radio_state;
    int           area_code;
    //char           cell_id[5];
    int cell_id;
    int           base_port;

    /* SMS */
    int           wait_sms;

    /* SIM card */
    ASimCard      sim;
    ASimCard      sim2;

    /* voice and data network registration */
    ARegistrationUnsolMode   voice_mode;
    ARegistrationState       voice_state;
    ARegistrationUnsolMode   data_mode;
    ARegistrationState       data_state;
    AGprsNetworkType         data_network;

    /* operator names */
    AOperatorSelection  oper_selection_mode;
    ANameIndex          oper_name_index;
    int                 oper_index;
    int                 oper_count;
    AOperatorRec        operators[ MAX_OPERATORS ];

    /* data connection contexts */
    ADataContextRec     data_contexts[ MAX_DATA_CONTEXTS ];

    /* active calls */
    AVoiceCallRec       calls[ MAX_CALLS ];
    int                 call_count;

    /* unsolicited callback */  /* XXX: TODO: use this */
    AModemUnsolFunc     unsol_func;
    void*               unsol_opaque;

    SmsReceiver         sms_receiver;

    int                 out_size;
   char                out_buff[1024];
   char	sms_message[1024];
   char	urc_buf[1024];
   
   /*support Gemini*/
   //sim1 ==1 ,sim2 ==2 ,none = 0
   int sims, sms;
   char *sims_ch;// AT come from which sim card

   /* adta connection state */
   ANetworkState data_sim;
   char* data_sim_ch;//which sim card is the data connection   
   
   SysTimer timer;
} AModemRec;


static void
acall_set_state( AVoiceCall    call, ACallState  state )
{

    if (state != call->call.state)
    {
        if (call->is_remote)
        {
            const char*  number = call->call.number;
            int          port   = call->modem->base_port;
            int sim = amodem_gemini_get_sims(call->modem);

            switch (state) {
                case A_CALL_HELD:
                    remote_call_other_gemini( number, port, REMOTE_CALL_HELD,sim);
                    break;

                case A_CALL_ACTIVE:
			if( A_CALL_HELD == call->call.state){
				remote_call_other_gemini( number, port, REMOTE_CALL_HELD,sim);
				break;
		 	} 
                    remote_call_other_gemini( number, port, REMOTE_CALL_ACCEPTED,sim);
                    break;

                default: 
		      break;
            }
        }
        call->call.state = state;
    }
}

static void
amodem_unsol( AModem  modem, const char* format, ... )
{
    if (modem->unsol_func) {
        va_list  args;
        va_start(args, format);
        vsnprintf( modem->out_buff, sizeof(modem->out_buff), format, args );
        va_end(args);
        modem->unsol_func( modem->unsol_opaque, modem->out_buff );
    }
}

void
amodem_receive_sms( AModem  modem, SmsPDU  sms )
{
	 printf("[Emu] amodem_receive_sms\r\n");
#define  SMS_UNSOL_HEADER_SIM1  "A+CMT: 21\r\n"
#define  SMS_UNSOL_HEADER_SIM2  "B+CMT: 21\r\n"

	int ll = sizeof(SMS_UNSOL_HEADER_SIM2);

    	if (modem->unsol_func) {
	        int    len, max;
	        char*  p;
		switch(amodem_gemini_get_sms(modem)){
			case 1:
				strcpy( modem->out_buff, SMS_UNSOL_HEADER_SIM1 );
				break;
			case 2:
				strcpy( modem->out_buff, SMS_UNSOL_HEADER_SIM2 );
				break;
			default:
				break;
		}
		printf("[Emu] amodem_receive_sms modem->out_buff %s\r\n",modem->out_buff);
		amodem_gemini_set_sms(modem,0);
	       p   = modem->out_buff + (ll-1);
		printf("[Emu] amodem_receive_sms %s\r\n",p);
	        max = sizeof(modem->out_buff) - 3 - (ll-1);
	        len = smspdu_to_hex( sms, p, max );
			printf("[Emu] amodem_receive_sms %s\r\n",p);
	        if (len > max) /* too long */
	            return;
	        p[len]   = '\r';
	        p[len+1] = '\n';
	        p[len+2] = 0;
		 printf("[Emu] amodem_receive_sms %s\r\n",p);
	        R( "SMS>> %s\n", p );
		 printf("[Emu] amodem_receive_sms %s\r\n",modem->out_buff);
	        modem->unsol_func( modem->unsol_opaque, modem->out_buff );
    }
}

static const char*
amodem_printf( AModem  modem, const char*  format, ... )
{
    va_list  args;
    va_start(args, format);
    vsnprintf( modem->out_buff, sizeof(modem->out_buff), format, args );
    va_end(args);
    if( memcmp(modem->out_buff, "+CSQ", 4)){
	  printf("[Emu] amodem_printf %s\r\n",modem->out_buff);
    }
    return modem->out_buff;
}

static void
amodem_begin_line( AModem  modem )
{
    modem->out_size = 0;
}

static void
amodem_add_line( AModem  modem, const char*  format, ... )
{
    va_list  args;
    va_start(args, format);
    modem->out_size += vsnprintf( modem->out_buff + modem->out_size,
                                  sizeof(modem->out_buff) - modem->out_size,
                                  format, args );
    va_end(args);
}

static const char*
amodem_end_line( AModem  modem )
{
    modem->out_buff[ modem->out_size ] = 0;
    return modem->out_buff;
}

static void
amodem_reset( AModem  modem )
{
    modem->radio_state = A_RADIO_STATE_OFF;
    modem->wait_sms    = 0;

    modem->oper_name_index     = 2;
    modem->oper_selection_mode = A_SELECTION_AUTOMATIC;
    modem->oper_index          = 0;
    modem->oper_count          = 2;

    modem->area_code = 1179;
    //modem->cell_id   = "62C8";
    modem->cell_id   = -1;
    strcpy( modem->operators[0].name[0], OPERATOR_HOME_NAME );
    strcpy( modem->operators[0].name[1], OPERATOR_HOME_NAME );
    strcpy( modem->operators[0].name[2], OPERATOR_HOME_MCCMNC );

    modem->operators[0].status        = A_STATUS_AVAILABLE;

    strcpy( modem->operators[1].name[0], OPERATOR_ROAMING_NAME );
    strcpy( modem->operators[1].name[1], OPERATOR_ROAMING_NAME );
    strcpy( modem->operators[1].name[2], OPERATOR_ROAMING_MCCMNC );

    modem->operators[1].status        = A_STATUS_AVAILABLE;

    modem->voice_mode   = A_REGISTRATION_UNSOL_ENABLED_FULL;
    modem->voice_state  = A_REGISTRATION_HOME;
    modem->data_mode    = A_REGISTRATION_UNSOL_ENABLED_FULL;
    modem->data_state   = A_REGISTRATION_HOME;
    modem->data_network = A_GPRS_NETWORK_UMTS;
}

static AModemRec   _android_modem[1];

//answer a MT call.(ATA)
static void create_answer_mo(AModem modem,ACall acall)
{
       amodem_unsol( modem, "%c+ECPI: %d,132,,,,,\"\",,\"\"\r",modem->sims_ch,acall->id);
	amodem_unsol( modem, "%c+ECPI: %d,6,0,1,1,0,\"%s\",129,\"\"\r",modem->sims_ch,acall->id, acall->number);
}

//answered. supported by two emulator
static void create_answer_mt(AModem modem,ACall acall)
{
	create_answer_mo(modem,acall);
}

static void create_hangup(AModem modem,ACall acall)
{
	amodem_unsol( modem, "%c+ECPI: %d,134,,,,,\"\",,\"\"\r",modem->sims_ch,acall->id );
	amodem_unsol( modem, "%c+ECPI: %d,1,0,0,,,\"\",,\"\",0\r",modem->sims_ch,acall->id );
	amodem_unsol( modem, "%c+ECPI: %d,129,0,0,,,\"\",,\"\",0\r",modem->sims_ch,acall->id );
	amodem_unsol( modem, "%c+ECPI: %d,133,,,,,\"\",,\"\"\r",modem->sims_ch,acall->id );
}

static void create_hangup_mt(AModem modem,ACall acall)
{
	create_hangup( modem,acall);
}

static void
create_call_alert_single(AVoiceCall vcall,ACall acall )
{
	AModem modem = vcall->modem;
	amodem_unsol( modem, "%c+CIEV: 5, 1\r",modem->sims_ch );
	amodem_unsol( modem, "%c+ECPI: %d,130,0,0,0,0,\"%s\",129,\"\"\r",modem->sims_ch,acall->id,acall->number );
	amodem_unsol( modem, "%c+CIEV: 2, 0\r",modem->sims_ch);
	amodem_unsol( modem, "%c+ECPI: %d,3,0,0,0,0,\"%s\",129,\"\"\r" ,modem->sims_ch,acall->id,acall->number);
	amodem_unsol( modem, "%c+ECPI: %d,4,0,1,0,0,\"%s\",129,\"\"\r" ,modem->sims_ch,acall->id,acall->number);
	amodem_unsol( modem, "%c+ECPI: %d,2,1,1,0,0,\"%s\",129,\"\"\r" ,modem->sims_ch,acall->id,acall->number);
}

static void
create_call_alert_double(AVoiceCall vcall,ACall call )
{
	AModem modem = vcall->modem;
	amodem_unsol( modem, "%c+ECPI: %d,130,0,0,0,0,\"%s\",129,\"\"\r",modem->sims_ch,call->id,call->number );
	amodem_unsol( modem, "%c+ECPI: %d,3,0,0,0,0,\"%s\",129,\"\"\r" ,modem->sims_ch,call->id,call->number);
	amodem_unsol( modem, "%c+CSSI: 1\r",modem->sims_ch);
	amodem_unsol( modem, "%c+ECPI: %d,2,1,1,0,0,\"%s\",129,\"\"\r" ,modem->sims_ch,call->id,call->number);
	amodem_unsol( modem, "%c+COLP: \"%s\",129,\"\",0\r",modem->sims_ch,call->number);	
}

//it is a mo call.(ATD)
static void create_call_mo(AVoiceCall vcall,ACall acall )
{	
	AModem modem = vcall->modem;
	
	if(modem->call_count>1){
		int nn=0;
		 for (nn = 0; nn < modem->call_count; nn++) {
	                AVoiceCall vcall = modem->calls + nn;
	                ACall call  = &vcall->call;
	                if (call->mode != A_CALL_VOICE)
	                    continue;
			  switch(call->state){
				case A_CALL_ACTIVE:
					amodem_unsol( modem, "%c+ECPI: %d,131,,,,,\"\",,\"\"\r",modem->sims_ch,call->id);
				  	acall_set_state( vcall, A_CALL_HELD );
					break;
				case A_CALL_INCOMING:
					//incoming always is the last call.
	                	       //handle the two MO CALLs
					create_call_alert_double(vcall,call );
					amodem_unsol( modem, "%c+ECPI: %d,132,,,,,\"\",,\"\"\r",modem->sims_ch,call->id);
		                	amodem_unsol( modem, "%c+ECPI: %d,6,0,1,0,0,\"%s\",129,\"\"\r",modem->sims_ch ,call->id,call->number);
				     	acall_set_state( vcall, A_CALL_ACTIVE );
					break;
				case A_CALL_ALERTING:
					create_call_alert_double(vcall,call);
					break;
				default:
					break;
			  }
	       }
	}else{
		switch (acall->state) {
			case A_CALL_ALERTING:
				create_call_alert_single(vcall,acall );
				break;
			case A_CALL_INCOMING:
				acall_set_state( vcall, A_CALL_ACTIVE );
				create_call_alert_single(vcall,acall );
				amodem_unsol( modem, "%c+ECPI: %d,6,0,1,0,0,\"%s\",129,\"\"\r" ,modem->sims_ch,acall->id,acall->number);
				break;
			default:
				break;
		}
	
	}
}

//it is a MT call.(RING)
static void create_call_mt(AModem modem,ACall acall)
{
   if(	modem->call_count > 1){
   	amodem_unsol( modem, "%c+CCWA: \"%s\",129,1\r" ,modem->sims_ch,acall->number);
	amodem_unsol( modem, "%c+ECPI: %d,0,0,1,1,0,\"%s\",161,\"\"\r" ,modem->sims_ch,acall->id,acall->number);
   }else{
	amodem_unsol( modem, "%c+ECPI: %d,0,0,0,1,0,\"%s\",129,\"%d\"\r" ,modem->sims_ch,acall->id,acall->number,modem->call_count);
	amodem_unsol( modem, "%c+ECPI: %d,4,0,1,1,0,\"%s\",129,\"\"\r" ,modem->sims_ch,acall->id,acall->number);
   	amodem_unsol( modem, "%cRING\r", modem->sims_ch);
   }

}

//rejected
static void create_rejected(AModem modem,ACall acall)
{
	amodem_unsol( modem, "%c+ECPI: %d,1,0,0,,,\"\",,\"\",17\r",modem->sims_ch,acall->id);
	amodem_unsol( modem, "%cBUSY\r",modem->sims_ch,acall->id);
	amodem_unsol( modem, "%c+ECPI: %d,129,0,0,,,\"\",,\"\",0\r",modem->sims_ch,acall->id);
	amodem_unsol( modem, "%c+ECPI: %d,133,,,,,\"\",,\"\"\r",modem->sims_ch,acall->id);
	amodem_unsol( modem, "%c+CIEV: 5, 0\r",modem->sims_ch);
	amodem_unsol( modem, "%c+CEER: 17, CM_USER_BUSY\r",modem->sims_ch);
}

static void create_reject_mo(AModem modem)
{
}

static void create_holdon(AModem modem)
{
}

AModem
amodem_create( int  base_port, AModemUnsolFunc  unsol_func, void*  unsol_opaque )
{
    AModem  modem = _android_modem;

    amodem_reset( modem );
    modem->supportsNetworkDataType = 1;
    modem->base_port    = base_port;
    modem->unsol_func   = unsol_func;
    modem->unsol_opaque = unsol_opaque;
    modem->sim= asimcard_create();
    modem->sim2 = asimcard_create();
	
    modem->sims = 0;
    modem->data_sim = A_NETWORK_STATE_OFF;
    modem->sims_ch = NULL;
    modem->data_sim_ch = NULL;
    modem->sms = 0;
    return  modem;
}

void
amodem_set_legacy( AModem  modem )
{
    modem->supportsNetworkDataType = 0;
}

void
amodem_destroy( AModem  modem )
{
    asimcard_destroy( modem->sim );
    modem->sim = NULL;

    asimcard_destroy( modem->sim2 );
    modem->sim2 = NULL;
}

static int
amodem_has_network( AModem  modem )
{
    return !(modem->radio_state == A_RADIO_STATE_OFF   ||
             modem->oper_index < 0                  ||
             modem->oper_index >= modem->oper_count ||
             modem->oper_selection_mode == A_SELECTION_DEREGISTRATION );
}

ARadioState
amodem_get_radio_state( AModem modem )
{
    return modem->radio_state;
}

void
amodem_set_radio_state( AModem modem, ARadioState  state )
{
    modem->radio_state = state;
}

ASimCard
amodem_get_sim( AModem  modem )
{
    return  modem->sim;
}

ARegistrationState
amodem_get_voice_registration( AModem  modem )
{
    return modem->voice_state;
}

void
amodem_set_voice_registration( AModem  modem, ARegistrationState  state )
{
    modem->voice_state = state;

    if (state == A_REGISTRATION_HOME)
        modem->oper_index = OPERATOR_HOME_INDEX;
    else if (state == A_REGISTRATION_ROAMING)
        modem->oper_index = OPERATOR_ROAMING_INDEX;

    switch (modem->voice_mode) {
        case A_REGISTRATION_UNSOL_ENABLED:
            amodem_unsol( modem, "+CREG: %d,%d\r",
                          modem->voice_mode, modem->voice_state );//need sim card id
            break;

        case A_REGISTRATION_UNSOL_ENABLED_FULL:
            amodem_unsol( modem, "+CREG: %d,%d, \"%d\", \"%d\"\r",
                          modem->voice_mode, modem->voice_state,
                          modem->area_code, modem->cell_id );//need sim card id
            break;
			
        default:
	     break;
    }
}

ARegistrationState
amodem_get_data_registration( AModem  modem )
{
    return modem->data_state;
}

static const char*
amodem_set_data_registration_gemini(const char*  cmd, AModem  modem )
{
    if ( !memcmp(cmd, "+CGACT=", 7) ) {
		cmd += 7;

	switch(cmd[0]){
		case '0':
			modem->data_state = A_REGISTRATION_UNSOL_DISABLED;
			break;
		case '1':
			modem->data_state = A_REGISTRATION_UNSOL_ENABLED;
			break;
			
       	 default:
           	 ;
	}
    }
    return NULL;
}

static void sendUrc(void* _void)
{
	AModem  modem = _void;
	modem->unsol_func( modem->unsol_opaque, modem->urc_buf );
}

static void charCopy(char* ch, const char* format, ... )
{
	va_list  args;
       va_start(args, format);
       vsnprintf( ch, 1024, format, args );
       va_end(args);
	printf("[Emu] ch %s\r\n",ch);
}

static const char*
handleNetwork( const char*  cmd, AModem  modem )
{
    printf("[Emu] handleNetwork %s\r\n",cmd);
    if (CHAR_A == cmd[0]  || CHAR_B == cmd[0]){
		modem->data_sim_ch = cmd[0];
	}else{
		return "ERROR";
	}
    printf("[Emu] handleNetwork data_sim_ch = %c\r\n",modem->data_sim_ch);
    cmd += 1;
    printf("[Emu] handleNetwork %s\r\n",cmd);
    if ( !memcmp(cmd, "+EGTYPE=0,1", 11) ) {
	printf("[Emu] handleNetwork +EGTYPE=0,1 \r\n");
	cmd += 11;
	amodem_gemini_set_data_sim(modem,modem->data_sim_ch);
	printf("[Emu]+EGTYPE=0,1 %d\r\n",modem->data_sim);
	amodem_unsol( modem, "%c+CGEV: ME DETACH\r",modem->data_sim_ch);
	charCopy( modem->urc_buf, "%c+CGREG: 4\r",modem->data_sim_ch);
	printf("[Emu]modem->urc_buf %s\r\n",modem->urc_buf);
	modem->timer = sys_timer_create();
	sys_timer_set( modem->timer, sys_time_ms() + CALL_DELAY_DIAL,sendUrc, modem );
  	#if 0
	if(A_NETWORK_STATE_OFF == modem->data_sim){
		amodem_gemini_set_data_sim(modem,modem->data_sim_ch);
		printf("[Emu]A+EGTYPE=0,1 %d\r\n",modem->data_sim);
		amodem_unsol( modem, "%c+CGEV: ME DETACH\r",modem->data_sim_ch);
		if(CHAR_A == modem->data_sim_ch){
			
		}
          	//amodem_unsol( modem, "%c+CGEV: NW DEACT \"IP\", \"127.0.0.1\", 1\r",modem->data_sim_ch);
		charCopy( modem->urc_buf, "%c+CGREG: 4\r",temp[0]);
		printf("[Emu]modem->urc_buf %s\r\n",modem->urc_buf);
		modem->timer = sys_timer_create();
    		sys_timer_set( modem->timer, sys_time_ms() + CALL_DELAY_DIAL,sendUrc, modem );
		amodem_gemini_set_data_sim( modem, NULL);
	}else {
		printf("[Emu]handleNetwork %c\r\n",cmd[0]);
		charCopy( modem->urc_buf, "%c+CGREG: 1\r",cmd[0] );
		modem->timer = sys_timer_create();
    		sys_timer_set( modem->timer, sys_time_ms() + CALL_DELAY_DIAL,
                     sendUrc, modem );
		amodem_gemini_set_data_sim( modem, cmd[0]);
		printf("[Emu]modem->out_buff2 %s\r\n",modem->urc_buf);
       }
	#endif
    }
   else if(!memcmp(cmd, "+EGTYPE=0",9)){
   	printf("[Emu] handleNetwork +EGTYPE=0 \r\n");
	cmd+=9;
	if(cmd[0] == modem->data_sim_ch ){
		amodem_unsol( modem, "%c+CGEV: ME DETACH\r",cmd[0]);
	}
   }
   else if (!memcmp(cmd,"+EGTYPE=1",9)){
   	printf("[Emu] handleNetwork +EGTYPE=1 \r\n");
	cmd+=9;
		charCopy( modem->urc_buf, "%c+CGREG: 1\r",modem->data_sim_ch);
		printf("[Emu]modem->urc_buf %s\r\n",modem->urc_buf);
		modem->timer = sys_timer_create();
    		sys_timer_set( modem->timer, sys_time_ms() + CALL_DELAY_DIAL,
                    				sendUrc, modem );
		amodem_gemini_set_data_sim( modem, modem->data_sim_ch);
   }
 return NULL;
}

void
amodem_set_data_registration( AModem  modem, ARegistrationState  state )
{
 	if (CHAR_A == modem->data_sim_ch || CHAR_B == modem->data_sim_ch) {	
	     amodem_unsol( modem, "%c+CGREG: 4\r",modem->data_sim_ch);
	     amodem_gemini_set_data_sim( modem, NULL);
       }else if(NULL == modem->data_sim_ch){
	     amodem_unsol( modem, "%c+CGREG: 1\r",modem->data_sim_ch);
	     amodem_gemini_set_data_sim( modem, CHAR_A);
       }
#if 0
    modem->data_state = state;

    switch (modem->data_mode) {
        case A_REGISTRATION_UNSOL_ENABLED:
            amodem_unsol( modem, "+CGREG: %d,%d\r",
                          modem->data_mode, modem->data_state );
            break;

        case A_REGISTRATION_UNSOL_ENABLED_FULL:
            if (modem->supportsNetworkDataType)
                amodem_unsol( modem, "+CGREG: %d,%d,\"%04x\",\"%04x\",\"%04x\"\r",
                            modem->data_mode, modem->data_state,
                            modem->area_code, modem->cell_id,
                            modem->data_network );
            else
                amodem_unsol( modem, "+CGREG: %d,%d,\"%04x\",\"%04x\"\r",
                            modem->data_mode, modem->data_state,
                            modem->area_code, modem->cell_id );
            break;

        default:
            ;
    }
	#endif
}

void
amodem_set_data_network_type( AModem  modem, AGprsNetworkType   type )
{
    modem->data_network = type;
    amodem_set_data_registration( modem, modem->data_state );
}

int
amodem_get_operator_name ( AModem  modem, ANameIndex  index, char*  buffer, int  buffer_size )
{
    AOperator  oper;
    int        len;
    if ( (unsigned)modem->oper_index >= (unsigned)modem->oper_count ||
         (unsigned)index > 2 )
        return 0;

    oper = modem->operators + modem->oper_index;
    len  = strlen(oper->name[index]) + 1;

    if (buffer_size > len)
        buffer_size = len;

    if (buffer_size > 0) {
        memcpy( buffer, oper->name[index], buffer_size-1 );
        buffer[buffer_size] = 0;
    }
    return len;
}

/* reset one operator name from a user-provided buffer, set buffer_size to -1 for zero-terminated strings */
void
amodem_set_operator_name( AModem  modem, ANameIndex  index, const char*  buffer, int  buffer_size )
{
    AOperator  oper;
    int        avail;

    if ( (unsigned)modem->oper_index >= (unsigned)modem->oper_count ||
         (unsigned)index > 2 )
        return;
    oper = modem->operators + modem->oper_index;
    avail = sizeof(oper->name[0]);
    if (buffer_size < 0)
        buffer_size = strlen(buffer);
    if (buffer_size > avail-1)
        buffer_size = avail-1;
    memcpy( oper->name[index], buffer, buffer_size );
    oper->name[index][buffer_size] = 0;
}

int
amodem_get_call_count( AModem  modem )
{
    return modem->call_count;
}

ACall
amodem_get_call( AModem  modem, int  index )
{
    if ((unsigned)index >= (unsigned)modem->call_count)
        return NULL;

    return &modem->calls[index].call;
}

static AVoiceCall
amodem_alloc_call( AModem   modem )
{
    AVoiceCall  call  = NULL;
    int         count = modem->call_count;

    if (count < MAX_CALLS) {
        int  id;

        /* find a valid id for this call */
        for (id = 0; id < modem->call_count; id++) {
            int  found = 0;
            int  nn;
            for (nn = 0; nn < count; nn++) {
                if ( modem->calls[nn].call.id == (id+1) ) {
                    found = 1;
                    break;
                }
            }
            if (!found)
                break;
        }
        call  = modem->calls + count;
        call->call.id = id + 1;
        call->modem   = modem;
        modem->call_count += 1;
    }
    return call;
}

static void
amodem_free_call( AModem  modem, AVoiceCall  call )
{
    int  nn;

    if (call->timer) {
        sys_timer_destroy( call->timer );
        call->timer = NULL;
    }

    if (call->is_remote) {
	int sim = amodem_gemini_get_sims( modem);

	ACall acall  = &call->call;

	if( A_CALL_INCOMING == acall->state){
		remote_call_cancel_gemini( call->call.number, modem->base_port ,sim,REMOTE_CALL_BUSY);	
    	}else{
              remote_call_cancel_gemini( call->call.number, modem->base_port ,sim,REMOTE_CALL_HANGUP);
	}
        call->is_remote = 0;
    }

    for (nn = 0; nn < modem->call_count; nn++) {
        if ( modem->calls + nn == call )
            break;
    }
    assert( nn < modem->call_count );
    memmove( modem->calls + nn, modem->calls + nn + 1, (modem->call_count - 1 - nn)*sizeof(*call) );
    modem->call_count -= 1;
    //[Emu]TODO  free the sims  when not have call
    if (modem->call_count == 0){
	  amodem_gemini_set_sims( modem, 0);
    }
}

static AVoiceCall
amodem_find_call( AModem  modem, int  id )
{
    int  nn;

    for (nn = 0; nn < modem->call_count; nn++) {
        AVoiceCall call = modem->calls + nn;
        if (call->call.id == id)
            return call;
    }
    return NULL;
}

static void
amodem_send_calls_update( AModem  modem )
{
   /* despite its name, this really tells the system that the call
    * state has changed */
   //[Emu]TODO
   #if 0
   amodem_unsol( modem, "RING\r" );
   #endif
 }

int
amodem_add_inbound_call( AModem  modem, const char*  number )
{
    AVoiceCall  vcall = amodem_alloc_call( modem );
    ACall       call  = &vcall->call;
    int         len;

    if (call == NULL)
        return -1;

    call->dir   = A_CALL_INBOUND;
    call->state = A_CALL_INCOMING;
    call->anotherstate = ANOTHER_ACTIVE;
    call->mode  = A_CALL_VOICE;
    call->multi = 0;
	
    vcall->is_remote = (remote_number_string_to_port(number+1) > 0);
    if(vcall->is_remote){
		if(number[0] == '1'){
			vcall->from_sim = 1;
		}else if(number[0] == '2'){
			vcall->from_sim = 2;
		}else{
			vcall->is_remote = 0;
			vcall->from_sim = 0;
		}
	}
    len  = strlen(number);
    if (len >= sizeof(call->number))
        len = sizeof(call->number)-1;

    memcpy( call->number, number, len );
    call->number[len] = 0;
    create_call_mt( modem,call);
    return 0;
}

ACall
amodem_find_call_by_number( AModem  modem, const char*  number )
{
    AVoiceCall  vcall = modem->calls;
    AVoiceCall  vend  = vcall + modem->call_count;

    if (!number)
        return NULL;
    for ( ; vcall < vend; vcall++ )
        if ( !strcmp(vcall->call.number, number) )
            return &vcall->call;
    return  NULL;
}

int
amodem_update_call( AModem  modem, const char*  fromNumber, ACallState  state )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);

    if (vcall == NULL)
        return -1;
    
    acall_set_state( vcall, state );
    amodem_send_calls_update(modem);
    return 0;
}

int
amodem_accept_call( AModem  modem, const char*  fromNumber, ACallState  state )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);

    if (vcall == NULL)
        return -1;
    ACall call = &vcall->call;
    if( A_CALL_INCOMING == call->state){
	create_answer_mo(modem,call);
	acall_set_state( vcall, state );
	}else{
	return -1;
	}
    return 0;
}

int
amodem_accepted_call( AModem  modem, const char*  fromNumber, ACallState  state )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);

    if (vcall == NULL)
        return -1;
    ACall call = &vcall->call;
    if(vcall->is_remote && A_CALL_ALERTING == call->state){
		create_answer_mt(modem,call);
		acall_set_state( vcall, state );
	}else{
	return -1;
	}
    return 0;
}

int
amodem_hold_call( AModem  modem, const char*  fromNumber, ACallState  state )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);
    if (vcall == NULL)
        return -1;
    char * ch = handleHangup( "+CHLD=2",modem );
# if 0 
    ACall call = &vcall->call;
    if( A_CALL_ACTIVE == call->state){
	if (modem->sims > 0 ){
		amodem_unsol( modem, "A+ECPI: %d,131,,,,,\"\",,\"\"\r",call->id);
	}else if (0>modem->sims){
		amodem_unsol( modem, "B+ECPI: %d,131,,,,,\"\",,\"\"\r",call->id);
	}
	acall_set_state( vcall, A_CALL_HELD );
    }else if(A_CALL_HELD == call->state){
    	if (modem->sims > 0 ){
		amodem_unsol( modem, "A+ECPI: %d,132,,,,,\"\",,\"\"\r",call->id);
	}else if (0>modem->sims){
		amodem_unsol( modem, "B+ECPI: %d,132,,,,,\"\",,\"\"\r",call->id);
	}
	acall_set_state( vcall, A_CALL_ACTIVE );
    }
#endif
    return 0;
}

int
amodem_held_call( AModem  modem, const char*  fromNumber )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);

    if (vcall == NULL)
        return -1;
    ACall call = &vcall->call;
    if(ANOTHER_ACTIVE == call->anotherstate){
	amodem_unsol( modem, "%c+CSSU: 2\r",modem->sims_ch);
	call->anotherstate = ANOTHER_HOLD;
    }else if(ANOTHER_HOLD == call->anotherstate){
	amodem_unsol( modem, "%c+CSSU: 3\r",modem->sims_ch);
	call->anotherstate = ANOTHER_ACTIVE;
    }
    return 0;
}


int
amodem_cancel_call( AModem  modem, const char*  fromNumber )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);

    if (vcall == NULL)
        return -1;
    ACall call = &vcall->call;
    if( A_CALL_ACTIVE == call->state || A_CALL_HELD == call->state){
	 create_hangup(modem,call);
	 amodem_free_call( modem, vcall );
    }
    return 0;
}

int
amodem_canceled_call( AModem  modem, const char*  fromNumber )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);

    if (vcall == NULL)
        return -1;
    ACall call = &vcall->call;
    if( A_CALL_INCOMING == call->state ||A_CALL_ACTIVE == call->state || A_CALL_HELD == call->state){
	 create_hangup(modem,call);
	 vcall->is_remote = 0;
    	amodem_free_call( modem, vcall );
    }
    
    return 0;
}

int
amodem_reject_call( AModem  modem, const char*  fromNumber )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);

    if (vcall == NULL)
        return -1;
    ACall call = &vcall->call;
    if( A_CALL_INCOMING == call->state){
	 create_hangup(modem,call);
	 amodem_free_call( modem, vcall );
    }
    return 0;
}

int
amodem_rejected_call( AModem  modem, const char*  fromNumber )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, fromNumber);

    if (vcall == NULL)
        return -1;
    ACall call = &vcall->call;
    if(  A_CALL_ALERTING== call->state){
	 create_rejected(modem,call);
 	 vcall->is_remote = 0;
	 amodem_free_call( modem, vcall );
    }
    return 0;
}

int
amodem_disconnect_call( AModem  modem, const char*  number )
{
    AVoiceCall  vcall = (AVoiceCall) amodem_find_call_by_number(modem, number);

    if (!vcall)
        return -1;

    amodem_free_call( modem, vcall );
    amodem_send_calls_update(modem);
    return 0;
}


static const char*
unknownCommand( const char*  cmd, AModem  modem )
{
    modem=modem;
    fprintf(stderr, ">>> unknown command '%s'\n", cmd );
    return "ERROR: unknown command\r";
}

static const char*
handleRadioPower( const char*  cmd, AModem  modem )
{
	
	if(!strcmp( cmd, "+CFUN=0" )){
		modem->radio_state = A_RADIO_STATE_OFF;/* turn radio off */
	}else if( !strcmp( cmd, "+CFUN=1" )){
		modem->radio_state = A_RADIO_STATE_ON;/* turn radio on */
	}
	assert( !memcmp( cmd, "+EFUN=", 6 ) );
	cmd+=6;
	switch(cmd[0]){
		case '0':
			modem->radio_state = A_RADIO_STATE_OFF;
			break;
		case '1':
			modem->radio_state = A_RADIO_SIM1_ONLY_STATE_ON;
			amodem_unsol( modem, "A+CREG: 1, \"1179\", \"62C8\"\r" );
			break;
		case '2':
			modem->radio_state = A_RADIO_SIM2__ONLY_STATE_ON;
			amodem_unsol( modem, "B+CREG: 1, \"1179\", \"62C8\"\r" );
			break;
		case '3':
			modem->radio_state = A_RADIO_STATE_ON;
			//amodem_unsol( modem, "B+CREG: 1, \"1179\", \"62C8\"\r" );
			//amodem_unsol( modem, "A+CREG: 1, \"1179\", \"62C8\"\r" );
			break;
		case '4':
			//modem->radio_state = A_RADIO_STATE_OFF;
			break;
		default:
			break;
	}
   #if 0
    if ( !strcmp( cmd, "+CFUN=0" ) || !strcmp(cmd,"+EFUN=4") ){
        /* turn radio off */
        modem->radio_state = A_RADIO_STATE_OFF;
    } else if ( !strcmp( cmd, "+CFUN=1" ) || !strcmp(cmd,"+EFUN=3") ){
        /* turn radio on */
        modem->radio_state = A_RADIO_STATE_ON;
    } else if (( !strcmp( cmd, "+EFUN=1" ))){
	 modem->radio_state = A_RADIO_SIM1_ONLY_STATE_ON;
    }else if (( !strcmp( cmd, "+EFUN=2" ))){
	 modem->radio_state = A_RADIO_SIM2__ONLY_STATE_ON;
    }
  #endif
    return NULL;
}

static const char*
handleRadioPowerReq( const char*  cmd, AModem  modem )
{
    //TODO have a bug,should judge sim1 or sim2.
    if (modem->radio_state == A_RADIO_STATE_OFF)
       return "+CFUN: 0";
    else
	return "+CFUN: 1";	
       // return "+CFUN: 0";TODO need modify
}




//get sim card ststus
static const char*
handleSIMStatusReq( const char*  cmd, AModem  modem )
{
    const char*  answer = NULL;
	ASimStatus simstatus;
    if(!memcmp( "+CPINB?", cmd, 7 ))
    	{
    		simstatus= asimcard_get_status(modem->sim2);
		amodem_unsol( modem, "B+EIND: 2\r");
    	}else{
		 simstatus = asimcard_get_status(modem->sim);
		 amodem_unsol( modem, "A+EIND: 2\r");//phonebook is ready so we need the EIND: 1 (sms ready)
	}
    switch (simstatus) {
        case A_SIM_STATUS_ABSENT:    answer = "+CPIN: ABSENT"; break;
        case A_SIM_STATUS_READY:     answer = "+CPIN: READY"; break;
        case A_SIM_STATUS_NOT_READY: answer = "+CMERROR: NOT READY"; break;
        case A_SIM_STATUS_PIN:       answer = "+CPIN: SIM PIN"; break;
        case A_SIM_STATUS_PUK:       answer = "+CPIN: SIM PUK"; break;
        case A_SIM_STATUS_NETWORK_PERSONALIZATION: answer = "+CPIN: PH-NET PIN"; break;
        default:
            answer = "ERROR: internal error";
    }
    return answer;
}

static const char*
handleChangeOrEnterPIN( const char*  cmd, AModem  modem )
{
    assert( !memcmp( cmd, "+CPIN=", 6 ) );
    cmd += 6;
   amodem_unsol( modem, "+EIND: 2\r");
    switch (asimcard_get_status(modem->sim)) {
        case A_SIM_STATUS_ABSENT:
            return "+CME ERROR: SIM ABSENT";

        case A_SIM_STATUS_NOT_READY:
            return "+CME ERROR: SIM NOT READY";

        case A_SIM_STATUS_READY:
            /* this may be a request to change the PIN */
            {
                if (strlen(cmd) == 9 && cmd[4] == ',') {
                    char  pin[5];
                    memcpy( pin, cmd, 4 ); pin[4] = 0;

                    if ( !asimcard_check_pin( modem->sim, pin ) )
                        return "+CME ERROR: BAD PIN";

                    memcpy( pin, cmd+5, 4 );
                    asimcard_set_pin( modem->sim, pin );
                    return "+CPIN: READY";
                }
            }
            break;

        case A_SIM_STATUS_PIN:   /* waiting for PIN */
            if ( asimcard_check_pin( modem->sim, cmd ) )
                return "+CPIN: READY";
            else
                return "+CME ERROR: BAD PIN";

        case A_SIM_STATUS_PUK:
            if (strlen(cmd) == 9 && cmd[4] == ',') {
                char  puk[5];
                memcpy( puk, cmd, 4 );
                puk[4] = 0;
                if ( asimcard_check_puk( modem->sim, puk, cmd+5 ) )
                    return "+CPIN: READY";
                else
                    return "+CME ERROR: BAD PUK";
            }
            return "+CME ERROR: BAD PUK";

        default:
            return "+CPIN: PH-NET PIN";
    }

    return "+CME ERROR: BAD FORMAT";
}

static const char*
SetPINFromPUK( const char*  cmd, AModem  modem)
{
	assert( !memcmp( cmd, "+EPIN", 5 ) );
	cmd+=8;
	printf("[Emu] SetPINFromPUK2 \r\n");
	char puk[A_SIM_PIN_SIZE+1],*p;
		printf("[Emu] SetPINFromPUK3 \r\n");
		printf("[Emu] SetPINFromPUK3 %s\r\n",cmd);

	p=memccpy(puk,cmd,'\"',A_SIM_PIN_SIZE);
			printf("[Emu] SetPINFromPUK3 %s\r\n",puk);	
			printf("[Emu] SetPINFromPUK3 %d\r\n",strlen(puk));
	if(p){
		--p;
		*p = '\0';
	}else{
		puk[A_SIM_PIN_SIZE] = '\0';
	}
	
	printf("[Emu] password1 %s\r\n",puk);
	cmd+=strlen(puk)+3;
	char pin[A_SIM_PIN_SIZE+1];
	printf("[Emu] password1 %s\r\n",cmd);
	p=memccpy(pin,cmd,'\"',A_SIM_PIN_SIZE);
	if(p){
		--p;
		*p = '\0';
	}else{
		pin[A_SIM_PIN_SIZE] = '\0';
	}
	printf("[Emu] password1 %s\r\n",pin);
	if ( !asimcard_check_puk( modem->sim, puk,pin ) ){
             return "+CME ERROR: 16";
	}
	return NULL;
}

static const char*
SIMRetryTimes( const char*  cmd, AModem  modem)
{
	 return amodem_printf( modem, "+EPINC: %d, %d, %d, %d",
                                  asimcard_get_pin_retries(modem->sim),asimcard_get_pin_retries(modem->sim2),
                                  asimcard_get_puk_retries(modem->sim), asimcard_get_puk_retries(modem->sim2) );
}

static const char*
StartSIMLock(const char*  cmd, AModem  modem )
{
	assert( !memcmp( cmd, "+CPIN=", 6 ) );
	cmd+=7;
	char pin[A_SIM_PIN_SIZE+1],*p;	
	p=memccpy(pin,cmd,'\"',A_SIM_PIN_SIZE);
	if(p){
		--p;
		*p = '\0';
	}else{
		pin[A_SIM_PIN_SIZE] = '\0';
	}
	if ( !asimcard_check_pin( modem->sim, pin ) ){
              return "+CME ERROR: 16";
	}
	asimcard_set_status(modem->sim,  A_SIM_STATUS_READY);
	return NULL;
}

static const char*
SimCardLock(const char*  cmd, AModem  modem )
{
	assert( !memcmp( cmd, "+CLCK=\"SC\"", 6 ) );
	cmd+=11;
	//printf("[Emu] password1 %s\r\n",cmd);
	//printf("[Emu] password1 %d\r\n",strlen(cmd));
	char* pos1 = strchr(cmd,'\"');
	//printf("[Emu] password1 %s\r\n",pos1);
	int len=0;
	char pin[A_SIM_PIN_SIZE+1],*p;
	if(pos1){
		p = memccpy(pin,pos1+1,'\"',A_SIM_PIN_SIZE);
		printf("[Emu] password1 %s\r\n",pin);
		if(p){
			--p;
			*p = '\0';
		}else{
			pin[A_SIM_PIN_SIZE] = '\0';
		}
	}
	switch (cmd[0]) {
		case '0':
			if ( !asimcard_check_pin( modem->sim, pin ) ){
                     return "+CME ERROR: 16";
			}
			asimcard_set_startstatus(modem->sim,A_SIM_STATUS_READY);
			break;
		case '1':
			if ( !asimcard_check_pin( modem->sim, pin ) ){
                     return "+CME ERROR: 16";
			}
			asimcard_set_startstatus(modem->sim,A_SIM_STATUS_PIN);
			break;
		case '2':
			if (asimcard_get_status(modem->sim)==A_SIM_STATUS_READY) {
				return "+CLCK: 0";
			}else{
				return "+CLCK: 1";
			}
			break;
		default:
			break;
		}
	return NULL;
}


static const char*
SetSIMPassword( const char*  cmd, AModem  modem)
{
	assert( !memcmp( cmd, "+CPWD=\"SC\"", 10 ) );
	cmd+=11;
	char* pos1 = strchr(cmd,'\"');
	int len=0;
	if(pos1){
		char pin[A_SIM_PIN_SIZE+1],*p;
		p = memccpy(pin,pos1+1,'\"',A_SIM_PIN_SIZE);
		if(p){
			--p;
			*p = '\0';
		}else{
			pin[A_SIM_PIN_SIZE] = '\0';
		}
		if ( !asimcard_check_pin( modem->sim, pin ) ){
                       return "+CME ERROR: 16";
		}
		pos1+=strlen(pin)+2;
		pos1 = strchr(pos1,'\"');
		if(pos1){
			char newpin[A_SIM_PIN_SIZE+1],*pp;
			pp = memccpy(newpin,pos1+1,'\"',A_SIM_PIN_SIZE);
			if(pp){
				--pp;
				*pp = '\0';
			}else{
				pin[A_SIM_PIN_SIZE] = '\0';
			}
			asimcard_set_pin(modem->sim,newpin);
		}
	}
	return NULL;
}


static const char*
handleNetworkRegistration( const char*  cmd, AModem  modem )
{
    printf("[Emu]handleNetworkRegistration %s\r\n",cmd);
    if ( !memcmp( cmd, "+CREG", 5 ) ) {
        cmd += 5;
        if (cmd[0] == '?') {
		/*
            return amodem_printf( modem, "+CREG: %d,%d, \"%04x\", \"%s\"",
                                  modem->voice_mode, modem->voice_state,
                                  modem->area_code, modem->cell_id );
            */
            return "+CREG: 2, 1, \"1179\", \"62C8\", 3";    
        } else if (cmd[0] == '=') {
            switch (cmd[1]) {
                case '0':
                    modem->voice_mode  = A_REGISTRATION_UNSOL_DISABLED;
                    break;

                case '1':
                    modem->voice_mode  = A_REGISTRATION_UNSOL_ENABLED;
                    break;

                case '2':
                    modem->voice_mode = A_REGISTRATION_UNSOL_ENABLED_FULL;
                    break;

                case '?':
			return "+CREG: 2, 1, \"1179\", \"62C8\", 2";  
                    //return "+CREG: (0-2)";
		      //return ammodem_printf(modem,"+CREG: 2, 1, \"1179\", \"62C8\", 3");   

                default:
                    return "ERROR";
                    //return "ERROR: BAD COMMAND";
            }
        } else {
            assert( 0 && "unreachable" );
        }
    } 
   else if ( !memcmp( cmd, "A+CGREG" , 7)  ||!memcmp( cmd, "B+CGREG", 7 ) ) {
	 if (CHAR_A == cmd[0] || CHAR_B == cmd[0]){
		modem->data_sim_ch = cmd[0];
	 }else{
		return "ERROR";
	 }
        cmd += 7;

        if (cmd[0] == '?') {
		printf("[Emu]handleNetworkRegistration ANetworkState = %d ,data_sim_ch = %c\r\n",modem->data_sim,modem->data_sim_ch);	
			
		if (A_NETWORK_STATE_OFF == modem->data_sim){
			return "+CGREG: 1, 4";
		}else if( A_NETWORK_STATE_SIM1 == modem->data_sim&& CHAR_A == modem->data_sim_ch){
			return "+CGREG: 1, 1, \"1179\", \"62C8\", 2";
		}else if(A_NETWORK_STATE_SIM2 == modem->data_sim && CHAR_B == modem->data_sim_ch){
			return "+CGREG: 1, 1, \"1179\", \"62C8\", 3";
		}else{
			return "+CGREG: 1, 4";
		}
		# if 0
		switch (modem->data_mode) {
       		 case A_REGISTRATION_UNSOL_ENABLED:
			 case A_REGISTRATION_UNSOL_ENABLED_FULL:		
				return "+CGREG: 1, 1, \"1179\", \"62C8\", 3";
			case A_REGISTRATION_UNSOL_DISABLED:
				return "+CGREG: 1, 4";
		}
		
            if (modem->supportsNetworkDataType)
                return amodem_printf( modem, "+CGREG: %d,%d,\"%04x\",\"%04x\",\"%04x\"",
                                    modem->data_mode, modem->data_state,
                                    modem->area_code, modem->cell_id,
                                    modem->data_network );
            else
                return amodem_printf( modem, "+CGREG: %d,%d,\"%04x\",\"%04x\"",
                                    modem->data_mode, modem->data_state,
                                    modem->area_code, modem->cell_id );
		#endif
        } 
	else if (cmd[0] == '=') {
            printf("[Emu]handleNetworkRegistration CGREG= %d\r\n",cmd[1]);
            switch (cmd[1]) {
                case '0':
                    modem->data_mode  = A_REGISTRATION_UNSOL_DISABLED;
                    break;

                case '1':
                    modem->data_mode  = A_REGISTRATION_UNSOL_ENABLED;
			
                    break;

                case '2':
                    modem->data_mode = A_REGISTRATION_UNSOL_ENABLED_FULL;
                    break;

                case '?':
                    return "+CGREG: 1, 1, \"1179\", \"62C8\", 3";

                default:
		      return "ERROR";
                    //return "ERROR: BAD COMMAND";
            }
        }
	else {
            assert( 0 && "unreachable" );
        }
    }
    return NULL;
}

static const char* handleNetworkRegistrationUrc(const char*  cmd, AModem  modem)
{
#if 0
    AVoiceCall  vcall = amodem_alloc_call( modem );
    ACall       call  = &vcall->call;
    vcall->timer = sys_timer_create();
    
        sys_timer_set( vcall->timer, sys_time_ms() + CALL_DELAY_DIAL,
                   Urc, modem );
	#endif
	amodem_unsol( modem, "A+CREG: 1, \"1179\", \"62C8\"\r" );
	amodem_unsol( modem, "B+CREG: 1, \"1179\", \"62C8\"\r" );
	return NULL;
}



static const char*
handleSetDialTone( const char*  cmd, AModem  modem )
{
    /* XXX: TODO */
    return NULL;
}

static const char*
handleDeleteSMSonSIM( const char*  cmd, AModem  modem )
{
    /* XXX: TODO */
    return NULL;
}

static const char*
handleSIM_IO( const char*  cmd, AModem  modem )
{
    return asimcard_io( modem->sim, cmd );
}


static const char*
handleICCID(const char* cmd,AModem modem)
{
	switch(cmd[0]){
		case 'A':
			return "+ICCID: 89860011010746583662";
		case 'B':
			return "+ICCID: 89860110611104426632";
		default:
			return "OK";
	}
}


static const char*
handleOperatorSelection( const char*  cmd, AModem  modem )
{
    assert( !memcmp( "+COPS", cmd, 5 ) );
    cmd += 5;
    if (cmd[0] == '?') { /* ask for current operator */
        AOperator  oper = &modem->operators[ modem->oper_index ];

        if ( !amodem_has_network( modem ) )
        {
            /* this error code means "no network" */
            return amodem_printf( modem, "+CME ERROR: 30" );
        }

        oper = &modem->operators[ modem->oper_index ];

        if ( modem->oper_name_index == 2 )
            return amodem_printf( modem, "+COPS: %d,2,%s",
                                  modem->oper_selection_mode,
                                  oper->name[2] );

        return amodem_printf( modem, "+COPS: %d,%d,\"%s\"",
                              modem->oper_selection_mode,
                              modem->oper_name_index,
                              oper->name[ modem->oper_name_index ] );
    }
    else if (cmd[0] == '=' && cmd[1] == '?') {  /* ask for all available operators */
        const char*  comma = "+COPS: ";
        int          nn;
        amodem_begin_line( modem );
        for (nn = 0; nn < modem->oper_count; nn++) {
            AOperator  oper = &modem->operators[nn];
            amodem_add_line( modem, "%s(%d,\"%s\",\"%s\",\"%s\")", comma,
                             oper->status, oper->name[0], oper->name[1], oper->name[2] );
            comma = ", ";
        }
        return amodem_end_line( modem );
    }
    else if (cmd[0] == '=') {
        switch (cmd[1]) {
            case '0':
                modem->oper_selection_mode = A_SELECTION_AUTOMATIC;
                return NULL;

            case '1':
                {
                    int  format, nn, len, found = -1;

                    if (cmd[2] != ',')
                        goto BadCommand;
                    format = cmd[3] - '0';
                    if ( (unsigned)format > 2 )
                        goto BadCommand;
                    if (cmd[4] != ',')
                        goto BadCommand;
                    cmd += 5;
                    len  = strlen(cmd);
                    if (*cmd == '"') {
                        cmd++;
                        len -= 2;
                    }
                    if (len <= 0)
                        goto BadCommand;

                    for (nn = 0; nn < modem->oper_count; nn++) {
                        AOperator    oper = modem->operators + nn;
                        char*        name = oper->name[ format ];

                        if ( !memcpy( name, cmd, len ) && name[len] == 0 ) {
                            found = nn;
                            break;
                        }
                    }

                    if (found < 0) {
                        /* Selection failed */
                        return "+CME ERROR: 529";
                    } else if (modem->operators[found].status == A_STATUS_DENIED) {
                        /* network not allowed */
                        return "+CME ERROR: 32";
                    }
                    modem->oper_index = found;

                    /* set the voice and data registration states to home or roaming
                     * depending on the operator index
                     */
                    if (found == OPERATOR_HOME_INDEX) {
                        modem->voice_state = A_REGISTRATION_HOME;
                        modem->data_state  = A_REGISTRATION_HOME;
                    } else if (found == OPERATOR_ROAMING_INDEX) {
                        modem->voice_state = A_REGISTRATION_ROAMING;
                        modem->data_state  = A_REGISTRATION_ROAMING;
                    }
                    return NULL;
                }

            case '2':
                modem->oper_selection_mode = A_SELECTION_DEREGISTRATION;
                return NULL;

            case '3':
                {
                    int format;

                    if (cmd[2] != ',')
                        goto BadCommand;

                    format = cmd[3] - '0';
                    if ( (unsigned)format > 2 )
                        goto BadCommand;

                    modem->oper_name_index = format;
                    return NULL;
                }
            default:
                ;
        }
    }
BadCommand:
    return unknownCommand(cmd,modem);
}

static const char*
handleRequestOperator( const char*  cmd, AModem  modem )
{
    AOperator  oper;
    cmd=cmd;

    if ( !amodem_has_network(modem) )
        return "+CME ERROR: 30";

    oper = modem->operators + modem->oper_index;
    modem->oper_name_index = 2;
    return amodem_printf( modem, "+COPS: 0,0,\"%s\"\r"
                          "+COPS: 0,1,\"%s\"\r"
                          "+COPS: 0,2,\"%s\"",
                          oper->name[0], oper->name[1], oper->name[2] );
}

static const char*
handleSendSMStoSIM( const char*  cmd, AModem  modem )
{
    /* XXX: TODO */
    return "ERROR: unimplemented";
}

static const char*
handleSendSMS( const char*  cmd, AModem  modem )
{
    modem->wait_sms = 1;
    return "> ";
}

#if 0
static void
sms_address_dump( SmsAddress  address, FILE*  out )
{
    int  nn, len = address->len;

    if (address->toa == 0x91) {
        fprintf( out, "+" );
    }
    for (nn = 0; nn < len; nn += 2)
    {
        static const char  dialdigits[16] = "0123456789*#,N%";
        int  c = address->data[nn/2];

        fprintf( out, "%c", dialdigits[c & 0xf] );
        if (nn+1 >= len)
            break;

        fprintf( out, "%c", dialdigits[(c >> 4) & 0xf] );
    }
}

static void
smspdu_dump( SmsPDU  pdu, FILE*  out )
{
    SmsAddressRec    address;
    unsigned char    temp[256];
    int              len;

    if (pdu == NULL) {
        fprintf( out, "SMS PDU is (null)\n" );
        return;
    }

    fprintf( out, "SMS PDU type:       " );
    switch (smspdu_get_type(pdu)) {
        case SMS_PDU_DELIVER: fprintf(out, "DELIVER"); break;
        case SMS_PDU_SUBMIT:  fprintf(out, "SUBMIT"); break;
        case SMS_PDU_STATUS_REPORT: fprintf(out, "STATUS_REPORT"); break;
        default: fprintf(out, "UNKNOWN");
    }
    fprintf( out, "\n        sender:   " );
    if (smspdu_get_sender_address(pdu, &address) < 0)
        fprintf( out, "(N/A)" );
    else
        sms_address_dump(&address, out);
    fprintf( out, "\n        receiver: " );
    if (smspdu_get_receiver_address(pdu, &address) < 0)
        fprintf(out, "(N/A)");
    else
        sms_address_dump(&address, out);
    fprintf( out, "\n        text:     " );
    len = smspdu_get_text_message( pdu, temp, sizeof(temp)-1 );
    if (len > sizeof(temp)-1 )
        len = sizeof(temp)-1;
    fprintf( out, "'%.*s'\n", len, temp );
}
#endif


static const char*
handleSendSMSText( const char*  cmd, AModem  modem )
{
#if 1
    SmsAddressRec  address;
    char           number[16];
    int            numlen;
    int            len = strlen(cmd);
    SmsPDU         pdu;

    /* get rid of trailing escape */
    if (len > 0 && cmd[len-1] == 0x1a)
        len -= 1;

    pdu = smspdu_create_from_hex( cmd, len );
    
    if (pdu == NULL) {
        D("%s: invalid SMS PDU ?: '%s'\n", __FUNCTION__, cmd);
        return "+CMS ERROR: INVALID SMS PDU";
    }
    printf("[Emu] pud\r\n");
    if (smspdu_get_receiver_address(pdu, &address) < 0) {
        D("%s: could not get SMS receiver address from '%s'\n",
          __FUNCTION__, cmd);
        return "+CMS ERROR: BAD SMS RECEIVER ADDRESS";
    }
    do {
        int  index;

        numlen = sms_address_to_str( &address, number, sizeof(number) );
	 printf("[Emu] handleSendSMSText number %s\r\n",number);
        if (numlen > sizeof(number)-1)
            break;
        number[numlen] = 0;
	 char* temp = number+1;
        if ( remote_number_string_to_port( temp ) < 0 )
            break;
        if (modem->sms_receiver == NULL) {
            modem->sms_receiver = sms_receiver_create();
            if (modem->sms_receiver == NULL) {
                D( "%s: could not create SMS receiver\n", __FUNCTION__ );
                break;
            }
        }
	printf("[Emu] number2\r\n ");
        index = sms_receiver_add_submit_pdu( modem->sms_receiver, pdu );
        if (index < 0) {
            D( "%s: could not add submit PDU\n", __FUNCTION__ );
            break;
        }
        /* the PDU is now owned by the receiver */
        pdu = NULL;

        if (index > 0) {
            SmsAddressRec  from[1];
            char           temp[10];
            SmsPDU*        deliver;
            int            nn;
	     printf("[Emu] number3 %d\r\n ",amodem_gemini_get_sms(modem));
	     int address = (amodem_gemini_get_sms(modem))*10000 +modem->base_port ;
	     printf("[Emu] handleSendSMSText2 address %d\r\n",address); 
	     amodem_gemini_set_sms( modem,0);
            sprintf( temp, "%d", address );
            sms_address_from_str( from, temp, strlen(temp) );
		printf("[Emu] handleSendSMSText number %s\r\n",number);
            deliver = sms_receiver_create_deliver( modem->sms_receiver, index, from );
            if (deliver == NULL) {
                D( "%s: could not create deliver PDUs for SMS index %d\n",
                   __FUNCTION__, index );
                break;
            }

            for (nn = 0; deliver[nn] != NULL; nn++) {
                if ( remote_call_sms_gemini( number, modem->base_port, deliver[nn] ,number[0]) < 0 ) {
                    D( "%s: could not send SMS PDU to remote emulator\n",
                       __FUNCTION__ );
                    break;
                }
            }
	 printf("[Emu] number 1\r\n");

            smspdu_free_list(deliver);
        }

    } while (0);

    if (pdu != NULL)
        smspdu_free(pdu);

#elif 1
    SmsAddressRec  address;
    char           number[16];
    int            numlen;
    int            len = strlen(cmd);
    SmsPDU         pdu;

    /* get rid of trailing escape */
    if (len > 0 && cmd[len-1] == 0x1a)
        len -= 1;

    pdu = smspdu_create_from_hex( cmd, len );
    if (pdu == NULL) {
        D("%s: invalid SMS PDU ?: '%s'\n", __FUNCTION__, cmd);
        return "+CMS ERROR: INVALID SMS PDU";
    }
    if (smspdu_get_receiver_address(pdu, &address) < 0) {
        D("%s: could not get SMS receiver address from '%s'\n",
          __FUNCTION__, cmd);
        return "+CMS ERROR: BAD SMS RECEIVER ADDRESS";
    }
    do {
        numlen = sms_address_to_str( &address, number, sizeof(number) );
        if (numlen > sizeof(number)-1)
            break;

        number[numlen] = 0;
        if ( remote_number_string_to_port( number ) < 0 )
            break;

        if ( remote_call_sms( number, modem->base_port, pdu ) < 0 )
        {
            D("%s: could not send SMS PDU to remote emulator\n",
              __FUNCTION__);
            return "+CMS ERROR: NO EMULATOR RECEIVER";
        }
    } while (0);
#else
    fprintf(stderr, "SMS<< %s\n", cmd);
    SmsPDU  pdu = smspdu_create_from_hex( cmd, strlen(cmd) );
    if (pdu == NULL) {
        fprintf(stderr, "invalid SMS PDU ?: '%s'\n", cmd);
    } else {
        smspdu_dump(pdu, stderr);
    }
#endif
    return "+CMGS: 0\rOK\r";
}

static const char*
handleMessage( const char*  cmd, AModem  modem )
{
	assert(!memcmp( cmd, "+CMGS", 5 ));
	printf("[Emu] message %s\r\n",cmd);
	cmd+=5;
	switch(cmd[0]){
		case 'A':
			amodem_gemini_set_sms(modem,1);
			break;
		case 'B':
			amodem_gemini_set_sms(modem,2);
			break;
	}
	char* pos1 = strchr(cmd,'\"');
	int len=0;
	if(pos1){
		char* pos2 = strchr(pos1+1,'\"');
		if(pos2){
			len = pos2 - pos1; 
			char* message = modem->sms_message;
			strncpy(message,pos1+1,len-1);
			handleSendSMSText(modem->sms_message, modem);
		}
	}
	return  "+CMGS: 0";
}


static const char*
handleListCurrentCalls( const char*  cmd, AModem  modem )
{
    int  nn;
    amodem_begin_line( modem );
    for (nn = 0; nn < modem->call_count; nn++) {
        AVoiceCall  vcall = modem->calls + nn;
        ACall       call  = &vcall->call;
        if (call->mode == A_CALL_VOICE)
            amodem_add_line( modem, "+CLCC: %d,%d,%d,%d,%d,\"%s\",%d\r\n",
                             call->id, call->dir, call->state, call->mode,
                             call->multi, call->number, 129 );
    }
    return amodem_end_line( modem );
}

static const char*
handleEndOfInit( const char*  cmd, AModem  modem )
{
    time_t       now = time(NULL);
    struct tm    utc, local;
    long         e_local, e_utc;
    long         tzdiff;
    char         tzname[64];

    tzset();

    utc   = *gmtime( &now );
    local = *localtime( &now );

    e_local = local.tm_min + 60*(local.tm_hour + 24*local.tm_yday);
    e_utc   = utc.tm_min   + 60*(utc.tm_hour   + 24*utc.tm_yday);

    if ( utc.tm_year < local.tm_year )
        e_local += 24*60;
    else if ( utc.tm_year > local.tm_year )
        e_utc += 24*60;

    tzdiff = e_local - e_utc;  /* timezone offset in minutes */

   /* retrieve a zoneinfo-compatible name for the host timezone
    */
    {
        char*  end = tzname + sizeof(tzname);
        char*  p = bufprint_zoneinfo_timezone( tzname, end );
        if (p >= end)
            strcpy(tzname, "Unknown/Unknown");

        /* now replace every / in the timezone name by a "!"
         * that's because the code that reads the CTZV line is
         * dumb and treats a / as a field separator...
         */
        p = tzname;
        while (1) {
            p = strchr(p, '/');
            if (p == NULL)
                break;
            *p = '!';
            p += 1;
        }
    }

   /* as a special extension, we append the name of the host's time zone to the
    * string returned with %CTZ. the system should contain special code to detect
    * and deal with this case (since it normally relied on the operator's country code
    * which is hard to simulate on a general-purpose computer
    */
    return amodem_printf( modem, "%%CTZV: %02d/%02d/%02d:%02d:%02d:%02d%c%d:%d:%s",
             (utc.tm_year + 1900) % 100, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec,
             (tzdiff >= 0) ? '+' : '-', (tzdiff >= 0 ? tzdiff : -tzdiff) / 15,
             (local.tm_isdst > 0),
             tzname );
}


static const char*
handleListPDPContexts( const char*  cmd, AModem  modem )
{

//#if 0
    switch (modem->data_mode) {
        case A_REGISTRATION_UNSOL_ENABLED:
	 case A_REGISTRATION_UNSOL_ENABLED_FULL:		
            return "+CGACT: 1, 0\r+CGACT: 2, 0\r+CGACT: 3, 0";
	case A_REGISTRATION_UNSOL_DISABLED:
            return "+CGACT: 1, 0\r+CGACT: 2, 0\r+CGACT: 3, 0";
	  default:
            return NULL;
    }
 return NULL;
//#endif
#if 0 
   // For data connection
    int  nn;
    assert( !memcmp( cmd, "+CGACT?", 7 ) );
    amodem_begin_line( modem );
    for (nn = 0; nn < MAX_DATA_CONTEXTS; nn++) {
        ADataContext  data = modem->data_contexts + nn;
        if (!data->active)
            continue;
        amodem_add_line( modem, "+CGACT: %d,%d\r", data->id, data->active );
    }
    return amodem_end_line( modem );
  #endif
}

static const char*
handleDefinePDP( const char*  cmd, AModem  modem )
{
	switch (modem->data_mode) {
        case A_REGISTRATION_UNSOL_ENABLED:
	 case A_REGISTRATION_UNSOL_ENABLED_FULL:		
            return "+CGDCONT: 1, \"IP\", \"cmnet\", \"\", 0, 0\r+CGDCONT: 2, \"IP\", \"internet\", \"\", 0, 0\r+CGDCONT: 3, \"IP\", \"internet\", \"\", 0, 0";
	case A_REGISTRATION_UNSOL_DISABLED:
            return "+CGDCONT: 1, \"IP\", \"cmnet\", \"\", 0, 0\r+CGDCONT: 2, \"IP\", \"internet\", \"\", 0, 0\r+CGDCONT: 3, \"IP\", \"internet\", \"\", 0, 0";
	  default:
            return NULL;
    }
 return NULL;
}
static const char*
handleDefinePDPContext( const char*  cmd, AModem  modem )
{
    assert( !memcmp( cmd, "+CGDCONT=", 9 ) );
    cmd += 9;
    if (cmd[0] == '?') {
        int  nn;
        amodem_begin_line(modem);
        for (nn = 0; nn < MAX_DATA_CONTEXTS; nn++) {
            ADataContext  data = modem->data_contexts + nn;
            if (!data->active)
                continue;
            amodem_add_line( modem, "+CGDCONT: %d,%s,\"%s\",,0,0\r\n",
                             data->id,
                             data->type == A_DATA_IP ? "IP" : "PPP",
                             data->apn );
        }
        return amodem_end_line(modem);
    } else {
        /* template is +CGDCONT=<id>,"<type>","<apn>",,0,0 */
        int              id = cmd[0] - '1';
        ADataType        type;
        char             apn[32];
        ADataContext     data;

        if ((unsigned)id > 3)
            goto BadCommand;

        if ( !memcmp( cmd+1, ",\"IP\",\"", 7 ) ) {
            type = A_DATA_IP;
            cmd += 8;
        } else if ( !memcmp( cmd+1, ",\"PPP\",\"", 8 ) ) {
            type = A_DATA_PPP;
            cmd += 9;
        } else
            goto BadCommand;

        {
            const char*  p = strchr( cmd, '"' );
            int          len;
            if (p == NULL)
                goto BadCommand;
            len = (int)( p - cmd );
            if (len > sizeof(apn)-1 )
                len = sizeof(apn)-1;
            memcpy( apn, cmd, len );
            apn[len] = 0;
        }

        data = modem->data_contexts + id;

        data->id     = id + 1;
        data->active = 1;
        data->type   = type;
        memcpy( data->apn, apn, sizeof(data->apn) );
    }
    return NULL;
BadCommand:
    return "ERROR";
    //return "ERROR: BAD COMMAND";
}


static const char*
handleStartPDPContext( const char*  cmd, AModem  modem )
{
    /* XXX: TODO: handle PDP start appropriately */
    /* for the moment, always return success */
#if 0
    AVoiceCall  vcall = amodem_alloc_call( modem );
    ACall       call  = (ACall) vcall;
    if (call == NULL) {
        return "ERROR: TOO MANY CALLS";
    }
    call->id    = 1;
    call->dir   = A_CALL_OUTBOUND;
    /* XXX: it would be better to delay this */
    call->state = A_CALL_ACTIVE;
    call->mode  = A_CALL_DATA;
    call->multi = 0;
    strcpy( call->number, "012345" );
#endif
    return NULL;
}


static void
remote_voice_call_event( void*  _vcall, int  success )
{
    AVoiceCall  vcall = _vcall;
    AModem      modem = vcall->modem;

    /* NOTE: success only means we could send the "gsm in new" command
     * to the remote emulator, nothing more */

    if (!success) {
        /* aargh, the remote emulator probably quitted at that point */
        amodem_free_call(modem, vcall);
	//[Emu]TODO
        //amodem_send_calls_update(modem);
    }
}


static void
voice_call_event( void*  _vcall )
{
    AVoiceCall  vcall = _vcall;
    ACall       call  = &vcall->call;


#if 0
   switch (call->state) {
        case A_CALL_DIALING:

            call->state = A_CALL_ALERTING;

            if (vcall->is_remote) {
                if ( remote_call_dial( call->number,
                                       vcall->modem->base_port,
                                       remote_voice_call_event, vcall ) < 0 )
                {
                   /* we could not connect, probably because the corresponding
                    * emulator is not running, so simply destroy this call.
                    * XXX: should we send some sort of message to indicate BAD NUMBER ? */
                    /* it seems the Android code simply waits for changes in the list   */
                    amodem_free_call( vcall->modem, vcall );
                }
            } else {
               /* this is not a remote emulator number, so just simulate
                * a small ringing delay */
                sys_timer_set( vcall->timer, sys_time_ms() + CALL_DELAY_ALERT,
                               voice_call_event, vcall );
            }
            break;

        case A_CALL_ALERTING:
            call->state = A_CALL_ACTIVE;
            break;
        default:
            assert( 0 && "unreachable event call state" );
   	}
   amodem_send_calls_update(vcall->modem);
#endif
    switch (call->state) {
	 case A_CALL_DIALING:

            call->state = A_CALL_INCOMING;
            if (vcall->is_remote) {
		  call->state =A_CALL_ALERTING;
		  int sim = amodem_gemini_get_sims(vcall->modem);

                if ( remote_call_dial_gemini( call->number,
                      vcall->modem->base_port ,remote_voice_call_event, 
                      vcall,sim) < 0 )
                {
                   /* we could not connect, probably because the corresponding
                    * emulator is not running, so simply destroy this call.
                    * XXX: should we send some sort of message to indicate BAD NUMBER ? */
                    /* it seems the Android code simply waits for changes in the list   */
		      create_hangup( vcall->modem, call);
                    amodem_free_call( vcall->modem, vcall );
                }
		}
             
            break;   
     
        default:
            assert( 0 && "unreachable event call state" );
    }

    create_call_mo(vcall,call);
}

static void
setSim(char * ch,AModem modem)
{
	if(CHAR_A == ch){
		amodem_gemini_set_sims(modem,SIM1);
	}else if(CHAR_B == ch){
		amodem_gemini_set_sims(modem,SIM2);
	}
}
static const char*
handleDial( const char*  cmd, AModem  modem )
{
    AVoiceCall  vcall = amodem_alloc_call( modem );
    ACall       call  = &vcall->call;
    int         len;

    if (call == NULL)
        return "ERROR: TOO MANY CALLS";

    assert( cmd[0] == 'D' );

    call->dir   = A_CALL_OUTBOUND;
    call->state = A_CALL_DIALING;
    call->anotherstate = ANOTHER_ACTIVE;
    call->mode  = A_CALL_VOICE;
    call->multi = 0;
    setSim(cmd[1],modem);
    cmd += 2;
    len  = strlen(cmd);
    if (len > 0 && cmd[len-1] == ';')
        len--;
    if (len >= sizeof(call->number))
        len = sizeof(call->number)-1;

    memcpy( call->number, cmd, len );
    call->number[len] = 0;
    char * is_port = call->number+1;
    vcall->is_remote = (remote_number_string_to_port(is_port) > 0);
    if (vcall->is_remote){
	if('1' != call->number[0] && '2' != call->number[0] ){
		vcall->is_remote = 0;
	}
    }
    vcall->timer = sys_timer_create();
	
    sys_timer_set( vcall->timer, sys_time_ms() + CALL_DELAY_DIAL,
                   voice_call_event, vcall );
    return NULL;
}


static const char*
handleAnswer( const char*  cmd, AModem  modem )
{
    int  nn;
    for (nn = 0; nn < modem->call_count; nn++) {
        AVoiceCall  vcall = modem->calls + nn;
        ACall       call  = &vcall->call;
        if (cmd[0] == 'A' ) {
		setSim(cmd[1],modem);
     
            if (call->state == A_CALL_INCOMING) {
		  create_answer_mo(modem,call);
                acall_set_state( vcall, A_CALL_ACTIVE );
            }
            else if (call->state == A_CALL_ACTIVE) {
                acall_set_state( vcall, A_CALL_HELD );
            }
        } else if (cmd[0] == 'H') {
            /* ATH: hangup, since user is busy */
            if (call->state == A_CALL_INCOMING) {

                amodem_free_call( modem, vcall );
                break;
            }
        }
    }
    return NULL;
}

static const char*
handleHangup( const char*  cmd, AModem  modem )
{
    if ( !memcmp(cmd, "+CHLD=", 6) ) { 

	AVoiceCall vcall = modem->calls + modem->call_count -1;
	ACall call  = &vcall->call;
	int is_incoming = 0;
	int count = modem->call_count;
	if(A_CALL_INCOMING == call->state){
	    is_incoming = 1;
	}

        int  nn;
        cmd += 6;
        switch (cmd[0]) {
            case '0':  /* release all held, and set busy for waiting calls */
                for (nn = 0; nn < modem->call_count; nn++) {
                    vcall = modem->calls + nn;
                    call  = &vcall->call;
                    if (call->mode != A_CALL_VOICE)
                        continue;
                    if ((call->state == A_CALL_HELD && 0 == is_incoming) ||
						call->state == A_CALL_INCOMING) {
			   create_hangup(modem,call);				
                        amodem_free_call(modem, vcall);
                        nn--;
                    }
                }
                break;

            case '1':
                if (cmd[1] == 0) { /* release all active, accept held one */
                    for (nn = 0; nn < modem->call_count; nn++) {
                        vcall = modem->calls + nn;
                        call  = &vcall->call;
                        if (call->mode != A_CALL_VOICE)
                            continue;
                        if (call->state == A_CALL_ACTIVE) {
				create_hangup(modem,call);		
			       amodem_free_call(modem, vcall);
                            nn--;		
                         }else if (call->state == A_CALL_INCOMING ) {
                             create_answer_mo(modem,call);
                            acall_set_state( vcall, A_CALL_ACTIVE );
                        }else if (A_CALL_HELD == call->state){
                            amodem_unsol( modem, "%c+ECPI: %d,132,,,,,\"\",,\"\"\r",modem->sims_ch,call->id );
				acall_set_state( vcall, A_CALL_ACTIVE );
			   }
                    }
			return"NO CARRIER";
                } else {  /* release specific call */
                    int  id = cmd[1] - '0';
                    AVoiceCall  vcall = amodem_find_call( modem, id );
                    if (vcall != NULL)
			   create_hangup(modem,&vcall->call);	
                        amodem_free_call( modem, vcall );
                }
                break;

            case '2':
                if (cmd[1] == 0) {  /* place all active on hold, accept held or waiting one */
                    for (nn = 0; nn < modem->call_count; nn++) {
                         vcall = modem->calls + nn;
                         call  = &vcall->call;
                        if (call->mode != A_CALL_VOICE)
                            continue;
                        if (call->state == A_CALL_ACTIVE) {
				amodem_unsol( modem, "%c+ECPI: %d,131,,,,,\"\",,\"\"\r",modem->sims_ch,call->id);
                            acall_set_state( vcall, A_CALL_HELD );
                        }
                        else if ( call->state == A_CALL_INCOMING ) {
                            amodem_unsol( modem, "%c+ECPI: %d,132,,,,,\"\",,\"\"\r",modem->sims_ch,call->id );
                            acall_set_state( vcall, A_CALL_ACTIVE );
                        }else if(call->state == A_CALL_HELD     ||
                                 call->state == A_CALL_WAITING){
                        	if (!is_incoming){
					amodem_unsol( modem, "%c+ECPI: %d,132,,,,,\"\",,\"\"\r",modem->sims_ch,call->id );
                           		acall_set_state( vcall, A_CALL_ACTIVE );
				}				
			   }
                    }
                } else {  /* place all active on hold, except a specific one */
                    int   id = cmd[1] - '0';
                    for (nn = 0; nn < modem->call_count; nn++) {
                        AVoiceCall  vcall = modem->calls + nn;
                        ACall       call  = &vcall->call;
                        if (call->mode != A_CALL_VOICE)
                            continue;
                        if (call->state == A_CALL_ACTIVE && call->id != id) {
				amodem_unsol( modem, "%c+ECPI: %d,131,,,,,\"\",,\"\"\r",modem->sims_ch,call->id );
                            acall_set_state( vcall, A_CALL_HELD );
                        }
                    }
                }
                break;

            case '3':  /* add a held call to the conversation */
                for (nn = 0; nn < modem->call_count; nn++) {
                    vcall = modem->calls + nn;
                    call  = &vcall->call;
                    if (call->mode != A_CALL_VOICE)
                        continue;
                    if (call->state == A_CALL_HELD) {
			   amodem_unsol( modem, "%c+ECPI: %d,132,,,,,\"\",,\"\"\r",modem->sims_ch,call->id );
                        acall_set_state( vcall, A_CALL_ACTIVE );
                        break;
                    }
                }
                break;

            case '4':  /* connect the two calls */
                for (nn = 0; nn < modem->call_count; nn++) {
                    vcall = modem->calls + nn;
                    call  = &vcall->call;
                    if (call->mode != A_CALL_VOICE)
                        continue;
                    if (call->state == A_CALL_HELD) {
                        acall_set_state( vcall, A_CALL_ACTIVE );
                        break;
                    }
                }
                break;
        }
    }
    else
	 return "ERROR";
        //return "ERROR: BAD COMMAND";

    return NULL;
}


/* a function used to deal with a non-trivial request */
typedef const char*  (*ResponseHandler)(const char*  cmd, AModem  modem);

static const struct {
    const char*      cmd;     /* command coming from libreference-ril.so, if first
                                 character is '!', then the rest is a prefix only */

    const char*      answer;  /* default answer, NULL if needs specific handling or
                                 if OK is good enough */

    ResponseHandler  handler; /* specific handler, ignored if 'answer' is not NULL,
                                 NULL if OK is good enough */
} sDefaultResponses[] =
{
//[Emu]TODO for MTK

	{ "+CSSN=1,1", NULL, NULL },
	{ "+CTZR=1", NULL, NULL },
	{ "+CMER=1,0,0,2,0", NULL, NULL },
	{ "+CGMR", "+CGMR: MAUI.10A.W10.47, 2010/11/18 17:22", NULL },
	{ "+EINFO=18", NULL, NULL },
	{ "+ECSQ=2", NULL, NULL },
	{ "+COLP=1", NULL, NULL },
	{ "+EGMR=0,9", "+EGMR: \"78\"", NULL },

	/*
	 *sim off/on and flightmode
	 */
	//{ "+EFUN=1", NULL, handleRadioPower },
	//{ "+EFUN=2", NULL, handleRadioPower },
	//{ "+EFUN=3", NULL, handleRadioPower },
	//{ "+EFUN=4", NULL, handleRadioPower },
	{ "+ESIMS", "+ESIMS: 1", NULL },
	{ "!+EFUN=", NULL, handleRadioPower },
       { "+CFUN=0", NULL, handleRadioPower },    /* see requestRadioPower() */
       { "+CFUN=1", NULL, handleRadioPower },
      { "+CFUN?", NULL, handleRadioPowerReq },    /* see isRadioOn() */
	//{ "+CFUN?", "+CFUN: 1", NULL },    /* see isRadioOn() */

	//network server (46000=china mobile)
	{ "+COPS=3,2;+COPS?", "+COPS: 0,2,\"46000\"", NULL },
	{ "+COPS?", "+COPS: 0,2,\"46000\"", NULL },

       /*
         * Network Registration 
         */
       //{ "+CREG=2", NULL, NULL },
       //{ "!+CREG", NULL, handleNetworkRegistration },
  	//{ "+CREG=2", NULL, handleNetworkRegistration },/* see requestRegistrationState() */
       { "!+CREG", NULL, handleNetworkRegistration },
       { "+CREG=2", NULL, NULL },
       {"!+CREG?", "+CREG: 2, 1, \"1179\", \"62C8\", 3", NULL },
       { "+CREG=1", NULL, handleNetworkRegistration },
       
       //{ "!+CGREG", NULL, handleNetworkRegistration },
      { "!A+CGREG", NULL, handleNetworkRegistration },
      { "!B+CGREG", NULL, handleNetworkRegistration },
      //{ "+CGREG=1", NULL, handleNetworkRegistration },

	//[Emu]TODO gprs
	{ "!A+EGTYPE", NULL, handleNetwork },
	{ "!B+EGTYPE", NULL, handleNetwork },
	/*
	 * sms
	 */
	//{ "!+CMGS", NULL, handleSendSMS },    /* see requestSendSMS() */
	//{ "!+CMGS", "+CMGS: 0", NULL },	
	{ "!+CMGS",NULL, handleMessage },	

     /*
       * call - hang up
	*/
    { "+CHLD=0", NULL, handleHangup },    /* see onRequest() */
    { "+CHLD=1", NULL, handleHangup },
    { "+CHLD=2", NULL, handleHangup },
    { "+CHLD=3", NULL, handleHangup },
    { "!+CHLD=", NULL, handleHangup },    /* see requestHangup() */
    { "!D", NULL, handleDial },  /*  see requestDial() the code says that success/error is ignored, the call state will be polled through +CLCC instead */
    { "AA", NULL, handleAnswer },  /* answer the call from sim1 */
    { "AB", NULL, handleAnswer },  /* answer the call from sim2*/
    { "H", NULL, handleAnswer },  /* user is busy */        
    //{ "+CHLD=0", "+CSSU: 3", handleHangup },   //this is waiting,another is hold
    //{ "+CHLD=0", "+CSSU: 2", handleHangup },   //this is forward,another is active from hold
    { "+CLCC", NULL, handleListCurrentCalls },     /* see requestGetCurrentCalls() */
    { "+CCWA=1", NULL, NULL },//TODO
    { "+ECPI=4294967295", NULL, NULL },


    /* see onRadioPowerOn() */
    { "%CPHS=1", NULL, NULL },
    { "%CTZV=1", NULL, NULL },

    /* see onSIMReady() */
    { "+CSMS=1", "+CSMS: 1, 1, 1", NULL },
    { "+CNMI=1,2,2,1,1", NULL, NULL },
    
    /* see requestOrSendPDPContextList() */
    { "+CGACT?", NULL, handleListPDPContexts },
    //[Emu]TODO_1
    { "+CGACT=0,1", NULL, amodem_set_data_registration_gemini },
    //{ "+CGACT=1,0", NULL, NULL },
    /* see requestOperator() */
   // { "+COPS=3,0;+COPS?;+COPS=3,1;+COPS?;+COPS=3,2;+COPS?", NULL, handleRequestOperator },

    /* see requestQueryNetworkSelectionMode() */
    //{ "!+COPS", NULL, handleOperatorSelection },

    /* see requestWriteSmsToSim() */
    { "!+CMGW=", NULL, handleSendSMStoSIM },
    { "B+ICCID?", NULL, handleICCID},
    { "A+ICCID?", NULL, handleICCID},
    /* see requestSignalStrength() */
    { "+CSQ", "+CSQ: 30,99", NULL },  /* XXX: TODO: implement variable signal strength and error rates */

    /* see requestSetupDefaultPDP() */
    { "%CPRIM=\"GMM\",\"CONFIG MULTISLOT_CLASS=<10>\"", NULL, NULL },
    { "%DATA=2,\"UART\",1,,\"SER\",\"UART\",0", NULL, NULL },

    /*
     * data connection
     */
    //{ "!+CGDCONT=", NULL, handleDefinePDPContext },
    { "+CGDCONT?", NULL, handleDefinePDP},
    { "!+CGQREQ=", NULL, NULL },
    { "+CGQMIN=1", NULL, NULL },
    { "+CGEREP=1,0", NULL, NULL },
    { "+CGPRCO=1,\"(null)\",\"(null)\",\"\",\"\"", NULL, NULL },
    { "+CGACT=1,1", NULL, NULL },
    { "+CGPADDR=1", "+CGPADDR: 1, \"127.0.0.1", NULL },
    { "+CGPRCO?", "+CGPRCO: 1, \"127.0.0.1\", \"127.0.0.1\"\r+CGPRCO: 2, \"172.21.120.6\", \"0.0.0.0\"\r+CGPRCO: 3, \"172.21.120.6\", \"0.0.0.0\"", NULL },
    { "+CGDATA=\"M-UPS\",1", NULL, NULL },

    { "D*99***1#", NULL, handleStartPDPContext },

    /* see requestSMSAcknowledge() */
    { "+CNMA=1", NULL, NULL },
    { "+CNMA=2", NULL, NULL },

    /* see requestSIM_IO() */
    { "!+CRSM=", NULL, handleSIM_IO },

    { "+CNMI?", "+CNMI: 1,2,2,1,1", NULL },

    { "!+VTS=", NULL, handleSetDialTone },
    { "+CIMI", OPERATOR_HOME_MCCMNC "000000000", NULL },   /* request internation subscriber identification number */
    //{ "+CIMI", "460078110018463", NULL },   /* request internation subscriber identification number */
    { "+CGSN", "000000000000000", NULL },   /* request model version */
    { "+CUSD=2",NULL, NULL }, /* Cancel USSD */
    { "+COPS=0", NULL, handleOperatorSelection }, /* set network selection to automatic */

    /*
     * sim card (PIN PUK)
     */
    { "!+CPIN=\"SC\"", NULL, handleChangeOrEnterPIN },
    { "+CPIN?", NULL, handleSIMStatusReq },    /* see getSIMStatus() */
    { "!+CPWD=\"SC\"",NULL,SetSIMPassword},
    { "!+CLCK=\"SC\"",NULL,SimCardLock},
    { "!+CPIN=",NULL,StartSIMLock},
    { "!+CMGD=", NULL, handleDeleteSMSonSIM }, /* delete SMS on SIM */
    { "+EPINC", NULL,SIMRetryTimes },
    { "!+EPIN", NULL, SetPINFromPUK },

    /* see initializeCallback() */
    { "E0Q0V1", NULL, NULL },
    { "S0=0", NULL, NULL },
    { "+CMEE=1", NULL, NULL },
    
    { "+CMOD=0", NULL, NULL },
    { "+CMUT=0", NULL, NULL },
    //{ "+CSSN=0,1", NULL, NULL },
    { "+COLP=0", NULL, NULL },
    //[Emu]TODO change  { "+CSCS=\"HEX\"", NULL, NULL },
    { "+CSCS=\"UCS2\"", NULL, NULL },
    { "+CUSD=1", NULL, NULL },
    { "+CGEREP=1,0", NULL, NULL },
    { "+CMGF=0", NULL, handleEndOfInit },  /* now is a goof time to send the current tme and timezone */
    { "%CPI=3", NULL, NULL },
    { "%CSTAT=1", NULL, NULL },

    /*
     * phonebook not done
     */
    { "+EIND", NULL, NULL },
    { "!+CPWD=\"P2\"",NULL, NULL },
    { "!+CLCK=\"FD\"",NULL, NULL },
    { "!+CPBS=\"FD\"",NULL, NULL },
    { "!+CPBW=2",NULL, NULL },
    { "!+EPIN2",NULL, NULL },

	//get modem top support 
    { "+EPSB?","+EPSB: 55", NULL },
    { "+CGDCONT=1,\"IP\",\"cmnet\",,0,0",NULL, NULL },


    /* end of list */
    {NULL, NULL, NULL}
};


#define  REPLY(str)  do { const char*  s = (str); R(">> %s\n", quote(s)); return s; } while (0)

const char*  amodem_send( AModem  modem, const char*  cmd )
{
    if( memcmp(cmd, "AT+CSQ", 6)){
	 printf("[Emu] command %s\r\n",cmd);
	}

    const char*  answer=NULL ;

    if ( modem->wait_sms != 0 ) {
        modem->wait_sms = 0;
        R( "SMS<< %s\n", quote(cmd) );
        answer = handleSendSMSText( cmd, modem );
	
        REPLY(answer);
    }

    /* everything that doesn't start with 'AT' is not a command, right ? */
    if ( cmd[0] != 'A' || cmd[1] != 'T' || cmd[2] == 0 ) {
        /* R( "-- %s\n", quote(cmd) ); */
        return NULL;
    }
    R( "<< %s\n", quote(cmd) );

    cmd += 2;

    /* TODO: implement command handling */
    {
        int  nn, found = 0;

        for (nn = 0; ; nn++) {
            const char*  scmd = sDefaultResponses[nn].cmd;

            if (!scmd) /* end of list */
                break;

            if (scmd[0] == '!') { /* prefix match */
                int  len = strlen(++scmd);

                if ( !memcmp( scmd, cmd, len ) ) {
                    found = 1;
                    break;
                }
            } else { /* full match */
                if ( !strcmp( scmd, cmd ) ) {
                    found = 1;
                    break;
                }
            }
        }

        if ( !found )
        {
            D( "** UNSUPPORTED COMMAND **\n" );
            REPLY( "ERROR" );
	     //REPLY( "ERROR: UNSUPPORTED" );
        }
        else
        {
            const char*      answer  = sDefaultResponses[nn].answer;
            ResponseHandler  handler = sDefaultResponses[nn].handler;

            if ( answer != NULL ) {
                REPLY( amodem_printf( modem, "%s\r@OK", answer ) );
            }

            if (handler == NULL) {
                REPLY( "OK" );
            }

            answer = handler( cmd, modem );
            if (answer == NULL)
                REPLY( "OK" );

            if ( !memcmp( answer, "> ", 2 )     ||
                 !memcmp( answer, "ERROR", 5 )  ||
                 !memcmp( answer, "+CME ERROR", 6 ) )
            {
                REPLY( answer );
            }

            if (answer != modem->out_buff)
                REPLY( amodem_printf( modem, "%s\r@OK", answer ) );

            strcat( modem->out_buff, "\r@OK" );
            REPLY( answer );
        }
    }
}

 int amodem_gemini_get_sims( AModem  modem )
{
	return modem->sims;
}

void amodem_gemini_set_sims( AModem  modem,int asims)
{
	modem->sims = asims;
	if(SIM1 == asims){
		modem->sims_ch = CHAR_A;
	}else if(SIM2 == asims){
		modem->sims_ch = CHAR_B;
	}else{
		modem->sims_ch = NULL;
	}
}

void amodem_gemini_set_sms( AModem  modem,int asms)
{
	modem->sms = asms;
}
int amodem_gemini_get_sms( AModem  modem)
{
	return modem->sms ;
}

void amodem_gemini_set_data_sim( AModem  modem,char* data_sim)
{
	if(CHAR_A == data_sim){
		modem->data_sim_ch = CHAR_A;
		modem->data_sim = A_NETWORK_STATE_SIM1;
	}else if(CHAR_B == data_sim){
		modem->data_sim_ch = CHAR_B;
		modem->data_sim = A_NETWORK_STATE_SIM2;
	}else{
		modem->data_sim_ch = NULL;
		modem->data_sim = A_NETWORK_STATE_OFF;
	}
}
char* amodem_gemini_get_data_sim( AModem  modem)
{
	return modem->data_sim_ch;
}

