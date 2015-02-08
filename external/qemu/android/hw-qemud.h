
#ifndef _android_qemud_h
#define _android_qemud_h

#include "qemu-common.h"



extern void  android_qemud_init( void );

extern CharDriverState*  android_qemud_get_cs( void );

extern int  android_qemud_get_channel( const char*  name, CharDriverState* *pcs );

extern int  android_qemud_set_channel( const char*  name, CharDriverState*  peer_cs );

/* list of known qemud channel names */
#define  ANDROID_QEMUD_GSM      "gsm"
#define  ANDROID_QEMUD_GPS      "gps"
#define  ANDROID_QEMUD_CONTROL  "control"
#define  ANDROID_QEMUD_SENSORS  "sensors"


typedef struct QemudClient   QemudClient;
typedef struct QemudService  QemudService;


typedef void (*QemudClientClose)( void*  opaque );

typedef void (*QemudClientRecv) ( void*  opaque, uint8_t*  msg, int  msglen, QemudClient*  client );

extern QemudClient*  qemud_client_new( QemudService*     service,
                                       int               channel_id,
                                       void*             clie_opaque,
                                       QemudClientRecv   clie_recv,
                                       QemudClientClose  clie_close );

extern void           qemud_client_set_framing( QemudClient*  client, int  enabled );

extern void   qemud_client_send ( QemudClient*  client, const uint8_t*  msg, int  msglen );

extern void   qemud_client_close( QemudClient*  client );


typedef QemudClient*  (*QemudServiceConnect)( void*   opaque, QemudService*  service, int  channel );

extern QemudService*  qemud_service_register( const char*          serviceName,
                                              int                  max_clients,
                                              void*                serv_opaque,
                                              QemudServiceConnect  serv_connect );

extern void           qemud_service_broadcast( QemudService*   sv,
                                               const uint8_t*  msg,
                                               int             msglen );

#endif /* _android_qemud_h */
