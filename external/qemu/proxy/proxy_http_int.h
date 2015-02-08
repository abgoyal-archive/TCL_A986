
#ifndef _PROXY_HTTP_INT_H
#define _PROXY_HTTP_INT_H

#include "proxy_http.h"
#include "proxy_int.h"

/* the HttpService object */
typedef struct HttpService {
    ProxyService        root[1];
    SockAddress         server_addr;  /* server address and port */
    char*               footer;      /* the footer contains the static parts of the */
    int                 footer_len;  /* connection header, we generate it only once */
    char                footer0[512];
} HttpService;

/* create a CONNECT connection (for port != 80) */
extern ProxyConnection*  http_connector_connect(
                                HttpService*   service,
                                SockAddress*   address );

/* create a HTTP rewriting connection (for port == 80) */
extern ProxyConnection*  http_rewriter_connect(
                                HttpService*   service,
                                SockAddress*   address );


#endif /* _PROXY_HTTP_INT_H */
