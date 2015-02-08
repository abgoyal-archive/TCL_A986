

#ifndef __UENUM_H
#define __UENUM_H

#include "unicode/utypes.h"

struct UEnumeration;
/** structure representing an enumeration object instance @stable ICU 2.2 */
typedef struct UEnumeration UEnumeration;

U_STABLE void U_EXPORT2
uenum_close(UEnumeration* en);

U_STABLE int32_t U_EXPORT2
uenum_count(UEnumeration* en, UErrorCode* status);

U_STABLE const UChar* U_EXPORT2
uenum_unext(UEnumeration* en,
            int32_t* resultLength,
            UErrorCode* status);

U_STABLE const char* U_EXPORT2
uenum_next(UEnumeration* en,
           int32_t* resultLength,
           UErrorCode* status);

U_STABLE void U_EXPORT2
uenum_reset(UEnumeration* en, UErrorCode* status);

#endif
