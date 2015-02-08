
#ifndef LOCALSVC_H
#define LOCALSVC_H

#include "unicode/utypes.h"

#if U_LOCAL_SERVICE_HOOK
U_CAPI void* uprv_svc_hook(const char *what, UErrorCode *status);
#endif

#endif
