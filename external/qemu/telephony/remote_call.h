
#ifndef _REMOTE_CALL_H
#define _REMOTE_CALL_H

#include "sms.h"

/* convert a base console port into a remote phone number, -1 on error */
extern int         remote_number_from_port( int  port );

/* convert a remote phone number into a remote console port, -1 on error */
extern int         remote_number_to_port( int  number );

extern int         remote_number_string_to_port( const char*  number );

typedef void   (*RemoteResultFunc)( void*  opaque, int  success );

typedef enum {
    REMOTE_CALL_DIAL = 0,
    REMOTE_CALL_BUSY,
    REMOTE_CALL_HANGUP,
    REMOTE_CALL_HELD,
    REMOTE_CALL_ACCEPTED,
    REMOTE_CALL_SMS
} RemoteCallType;

extern  int     remote_call_dial( const char*       to_number,
                                  int               from_port,
                                  RemoteResultFunc  result_func,
                                  void*             result_opaque );

extern  int     remote_call_dial_gemini( const char*       to_number,
                                  int               from_port,
                                  RemoteResultFunc  result_func,
                                  void*             result_opaque,
                                  int sims);

/* call this function to send a SMS to a remote emulator */
extern int      remote_call_sms( const char*   number, int  from_port, SmsPDU  pdu );
extern int      remote_call_sms_gemini( const char*   number, int  from_port, SmsPDU  pdu,int from_sim );

/* call this function to indicate that you're busy to a remote caller */
extern void     remote_call_other( const char*  to_number, int  from_port, RemoteCallType  type );

extern void     remote_call_other_gemini( const char*  to_number, int  from_port, RemoteCallType  type,int sims );

extern void     remote_call_cancel( const char*  to_number, int from_port );

#endif /* _REMOTE_CALL_H */
