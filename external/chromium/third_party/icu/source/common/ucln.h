
#ifndef __UCLN_H__
#define __UCLN_H__

#include "unicode/utypes.h"


typedef enum ECleanupLibraryType {
    UCLN_START = -1,
    UCLN_CUSTOM,    /* Custom is for anyone else. */
    UCLN_CTESTFW,
    UCLN_LAYOUTEX,
    UCLN_LAYOUT,
    UCLN_IO,
    UCLN_I18N,
    UCLN_COMMON /* This must be the last one to cleanup. */
} ECleanupLibraryType;

U_CDECL_BEGIN
typedef UBool U_CALLCONV cleanupFunc(void);
U_CDECL_END

U_CAPI void U_EXPORT2 ucln_registerCleanup(ECleanupLibraryType type,
                                           cleanupFunc *func);

#endif
