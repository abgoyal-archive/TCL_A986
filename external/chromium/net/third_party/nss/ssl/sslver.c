
/* Library identity and versioning */

#include "nss.h"

#if defined(DEBUG)
#define _DEBUG_STRING " (debug)"
#else
#define _DEBUG_STRING ""
#endif

const char __nss_ssl_rcsid[] = "$Header: NSS " NSS_VERSION _DEBUG_STRING
        "  " __DATE__ " " __TIME__ " $";
const char __nss_ssl_sccsid[] = "@(#)NSS " NSS_VERSION _DEBUG_STRING
        "  " __DATE__ " " __TIME__;
