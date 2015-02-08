

#ifndef INCLUDES_H
#define INCLUDES_H

/* Include possible build time configuration before including anything else */
#include "build_config.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#ifndef _WIN32_WCE
#ifndef CONFIG_TI_COMPILER
#include <signal.h>
#include <sys/types.h>
#endif /* CONFIG_TI_COMPILER */
#include <errno.h>
#endif /* _WIN32_WCE */
#include <ctype.h>
#include <time.h>

#ifndef CONFIG_TI_COMPILER
#ifndef _MSC_VER
#include <unistd.h>
#endif /* _MSC_VER */
#endif /* CONFIG_TI_COMPILER */

#ifndef CONFIG_NATIVE_WINDOWS
#ifndef CONFIG_TI_COMPILER
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#ifndef __vxworks
#ifndef __SYMBIAN32__
#include <sys/uio.h>
#endif /* __SYMBIAN32__ */
#include <sys/time.h>
#endif /* __vxworks */
#endif /* CONFIG_TI_COMPILER */
#endif /* CONFIG_NATIVE_WINDOWS */

#endif /* INCLUDES_H */
