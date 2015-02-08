
#ifndef _PROXY_HTTP_H
#define _PROXY_HTTP_H

#include "proxy_common.h"

extern int
proxy_http_setup( const char*         servername,
                  int                 servernamelen,
                  int                 serverport,
                  int                 num_options,
                  const ProxyOption*  options );

#endif /* END */
