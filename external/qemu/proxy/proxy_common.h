
#ifndef _PROXY_COMMON_H_
#define _PROXY_COMMON_H_

#include "sockets.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/select.h>
#endif

/* types and definitions used by all proxy connections */

typedef enum {
    PROXY_EVENT_NONE,
    PROXY_EVENT_CONNECTED,
    PROXY_EVENT_CONNECTION_REFUSED,
    PROXY_EVENT_SERVER_ERROR
} ProxyEvent;

/* event can't be NONE when this callback is called */
typedef void (*ProxyEventFunc)( void*  opaque, int  fd, ProxyEvent  event );

extern void  proxy_set_verbose(int  mode);


typedef enum {
    PROXY_OPTION_AUTH_USERNAME = 1,
    PROXY_OPTION_AUTH_PASSWORD,

    PROXY_OPTION_HTTP_NOCACHE = 100,
    PROXY_OPTION_HTTP_KEEPALIVE,
    PROXY_OPTION_HTTP_USER_AGENT,

    PROXY_OPTION_MAX

} ProxyOptionType;


typedef struct {
    ProxyOptionType  type;
    const char*      string;
    int              string_len;
} ProxyOption;


extern int   proxy_manager_add( SockAddress*         address,
                                SocketType           sock_type,
                                ProxyEventFunc       ev_func,
                                void*                ev_opaque );

extern void  proxy_manager_del( void*  ev_opaque );

extern void  proxy_manager_select_fill( int     *pcount, 
                                        fd_set*  read_fds, 
                                        fd_set*  write_fds, 
                                        fd_set*  err_fds);

/* this function is called to act on proxified connection sockets when network events arrive */
extern void  proxy_manager_poll( fd_set*  read_fds, 
                                 fd_set*  write_fds, 
                                 fd_set*  err_fds );

#endif /* END */
