

#ifndef ICUPLUGIMP_H
#define ICUPLUGIMP_H

#include "unicode/icuplug.h"

/*========================*/

U_INTERNAL void * U_EXPORT2
uplug_openLibrary(const char *libName, UErrorCode *status);

U_INTERNAL void U_EXPORT2
uplug_closeLibrary(void *lib, UErrorCode *status);

U_INTERNAL  char * U_EXPORT2
uplug_findLibrary(void *lib, UErrorCode *status);

/** @} */

/*========================*/

U_INTERNAL void U_EXPORT2
uplug_init(UErrorCode *status);

U_INTERNAL UPlugData* U_EXPORT2
uplug_getPlugInternal(int32_t n);

U_INTERNAL const char* U_EXPORT2
uplug_getPluginFile(void);

/** @} */

#endif
