
#ifndef _PROXY_INT_H
#define _PROXY_INT_H

#include "proxy_common.h"
#include "sockets.h"
#include "android/utils/stralloc.h"

extern int  proxy_log;

extern void
proxy_LOG(const char*  fmt, ...);

#define  PROXY_LOG(...)   \
    do { if (proxy_log) proxy_LOG(__VA_ARGS__); } while (0)


/* ProxySelect is used to handle events */

enum {
    PROXY_SELECT_READ  = (1 << 0),
    PROXY_SELECT_WRITE = (1 << 1),
    PROXY_SELECT_ERROR = (1 << 2)
};

typedef struct {
    int*     pcount;
    fd_set*  reads;
    fd_set*  writes;
    fd_set*  errors;
} ProxySelect;

extern void     proxy_select_set( ProxySelect*  sel,
                                  int           fd,
                                  unsigned      flags );

extern unsigned  proxy_select_poll( ProxySelect*  sel, int  fd );


/* sockets proxy manager internals */

typedef struct ProxyConnection   ProxyConnection;
typedef struct ProxyService      ProxyService;

/* free a given proxified connection */
typedef void              (*ProxyConnectionFreeFunc)   ( ProxyConnection*  conn );

/* modify the ProxySelect to tell which events to listen to */
typedef void              (*ProxyConnectionSelectFunc) ( ProxyConnection*  conn,
                                                         ProxySelect*      sel );

/* action a proxy connection when select() returns certain events for its socket */
typedef void              (*ProxyConnectionPollFunc)   ( ProxyConnection*  conn,
                                                         ProxySelect*      sel );


/* root ProxyConnection object */
struct ProxyConnection {
    int                 socket;
    SockAddress         address;  /* for debugging */
    ProxyConnection*    next;
    ProxyConnection*    prev;
    ProxyEventFunc      ev_func;
    void*               ev_opaque;
    ProxyService*       service;

    /* the following is useful for all types of services */
    char                name[64];    /* for debugging purposes */

    stralloc_t          str[1];      /* network buffer (dynamic) */
    int                 str_pos;     /* see proxy_connection_send() */
    int                 str_sent;    /* see proxy_connection_send() */
    int                 str_recv;    /* see proxy_connection_receive() */

    /* connection methods */
    ProxyConnectionFreeFunc    conn_free;
    ProxyConnectionSelectFunc  conn_select;
    ProxyConnectionPollFunc    conn_poll;

    /* rest of data depend on exact implementation */
};



extern void
proxy_connection_init( ProxyConnection*           conn,
                       int                        socket,
                       SockAddress*               address,
                       ProxyService*              service,
                       ProxyConnectionFreeFunc    conn_free,
                       ProxyConnectionSelectFunc  conn_select,
                       ProxyConnectionPollFunc    conn_poll );

extern void
proxy_connection_done( ProxyConnection*  conn );

extern void
proxy_connection_free( ProxyConnection*  conn,
                       int               keep_alive,
                       ProxyEvent        event );

/* status of data transfer operations */
typedef enum {
    DATA_ERROR     = -1,
    DATA_NEED_MORE =  0,
    DATA_COMPLETED =  1
} DataStatus;

extern DataStatus
proxy_connection_send( ProxyConnection*  conn, int  fd );

extern DataStatus
proxy_connection_receive( ProxyConnection*  conn, int  fd, int  wanted );

extern DataStatus
proxy_connection_receive_line( ProxyConnection*  conn, int  fd );

/* rewind the string buffer for a new operation */
extern void
proxy_connection_rewind( ProxyConnection*  conn );

extern int
proxy_base64_encode( const char*  src, int  srclen,
                     char*        dst, int  dstlen );

extern int
proxy_resolve_server( SockAddress*   addr,
                      const char*    servername,
                      int            servernamelen,
                      int            serverport );

/* a ProxyService is really a proxy server and associated options */

/* destroy a given proxy service */
typedef void              (*ProxyServiceFreeFunc)      ( void*  opaque );

typedef ProxyConnection*  (*ProxyServiceConnectFunc)( void*               opaque,
                                                      SocketType          socket_type,
                                                      const SockAddress*  address );

struct ProxyService {
    void*                      opaque;
    ProxyServiceFreeFunc       serv_free;
    ProxyServiceConnectFunc    serv_connect;
};

extern int
proxy_manager_add_service( ProxyService*  service );


#endif /* _PROXY_INT_H */
