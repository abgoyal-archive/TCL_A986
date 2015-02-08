
#ifndef __USET_IMP_H__
#define __USET_IMP_H__

#include "unicode/utypes.h"
#include "unicode/uset.h"

U_CDECL_BEGIN

typedef void U_CALLCONV
USetAdd(USet *set, UChar32 c);

typedef void U_CALLCONV
USetAddRange(USet *set, UChar32 start, UChar32 end);

typedef void U_CALLCONV
USetAddString(USet *set, const UChar *str, int32_t length);

typedef void U_CALLCONV
USetRemove(USet *set, UChar32 c);

typedef void U_CALLCONV
USetRemoveRange(USet *set, UChar32 start, UChar32 end);

struct USetAdder {
    USet *set;
    USetAdd *add;
    USetAddRange *addRange;
    USetAddString *addString;
    USetRemove *remove;
    USetRemoveRange *removeRange;
};
typedef struct USetAdder USetAdder;

U_CDECL_END

#endif

