
#ifndef __UENUMIMP_H
#define __UENUMIMP_H

#include "unicode/uenum.h"

U_CDECL_BEGIN


typedef void U_CALLCONV
UEnumClose(UEnumeration *en);

typedef int32_t U_CALLCONV
UEnumCount(UEnumeration *en, UErrorCode *status);

typedef const UChar* U_CALLCONV 
UEnumUNext(UEnumeration* en,
            int32_t* resultLength,
            UErrorCode* status);

typedef const char* U_CALLCONV 
UEnumNext(UEnumeration* en,
           int32_t* resultLength,
           UErrorCode* status);

typedef void U_CALLCONV 
UEnumReset(UEnumeration* en, 
            UErrorCode* status);


struct UEnumeration {
    /* baseContext. For the base class only. Don't touch! */
    void *baseContext;

    /* context. Use it for what you need */
    void *context;

    /** 
     * these are functions that will 
     * be used for APIs
     */
    /* called from uenum_close */
    UEnumClose *close;
    /* called from uenum_count */
    UEnumCount *count;
    /* called from uenum_unext */
    UEnumUNext *uNext;
    /* called from uenum_next */
    UEnumNext  *next;
    /* called from uenum_reset */
    UEnumReset *reset;
};

U_CDECL_END

U_CAPI const UChar* U_EXPORT2
uenum_unextDefault(UEnumeration* en,
            int32_t* resultLength,
            UErrorCode* status);

U_CAPI const char* U_EXPORT2
uenum_nextDefault(UEnumeration* en,
            int32_t* resultLength,
            UErrorCode* status);

#endif
