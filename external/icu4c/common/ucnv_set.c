
#include "unicode/utypes.h"
#include "unicode/uset.h"
#include "unicode/ucnv.h"
#include "ucnv_bld.h"
#include "uset_imp.h"

#if !UCONFIG_NO_CONVERSION

U_CAPI void U_EXPORT2
ucnv_getUnicodeSet(const UConverter *cnv,
                   USet *setFillIn,
                   UConverterUnicodeSet whichSet,
                   UErrorCode *pErrorCode) {
    /* argument checking */
    if(pErrorCode==NULL || U_FAILURE(*pErrorCode)) {
        return;
    }
    if(cnv==NULL || setFillIn==NULL || whichSet<UCNV_ROUNDTRIP_SET || UCNV_SET_COUNT<=whichSet) {
        *pErrorCode=U_ILLEGAL_ARGUMENT_ERROR;
        return;
    }

    /* does this converter support this function? */
    if(cnv->sharedData->impl->getUnicodeSet==NULL) {
        *pErrorCode=U_UNSUPPORTED_ERROR;
        return;
    }

    {
        USetAdder sa={
            NULL,
            uset_add,
            uset_addRange,
            uset_addString,
            uset_remove,
            uset_removeRange
        };
        sa.set=setFillIn;

        /* empty the set */
        uset_clear(setFillIn);

        /* call the converter to add the code points it supports */
        cnv->sharedData->impl->getUnicodeSet(cnv, &sa, whichSet, pErrorCode);
    }
}

#endif
