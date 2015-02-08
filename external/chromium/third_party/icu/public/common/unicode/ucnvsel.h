
#ifndef __ICU_UCNV_SEL_H__
#define __ICU_UCNV_SEL_H__

#include "unicode/uset.h"
#include "unicode/utypes.h"
#include "unicode/utf16.h"
#include "unicode/uenum.h"
#include "unicode/ucnv.h"


struct UConverterSelector;
typedef struct UConverterSelector UConverterSelector;
/** @} */

U_CAPI UConverterSelector* U_EXPORT2
ucnvsel_open(const char* const*  converterList, int32_t converterListSize,
             const USet* excludedCodePoints,
             const UConverterUnicodeSet whichSet, UErrorCode* status);

U_CAPI void U_EXPORT2
ucnvsel_close(UConverterSelector *sel);

U_CAPI UConverterSelector* U_EXPORT2
ucnvsel_openFromSerialized(const void* buffer, int32_t length, UErrorCode* status);

U_CAPI int32_t U_EXPORT2
ucnvsel_serialize(const UConverterSelector* sel,
                  void* buffer, int32_t bufferCapacity, UErrorCode* status);

U_CAPI UEnumeration * U_EXPORT2
ucnvsel_selectForString(const UConverterSelector* sel,
                        const UChar *s, int32_t length, UErrorCode *status);

U_CAPI UEnumeration * U_EXPORT2
ucnvsel_selectForUTF8(const UConverterSelector* sel,
                      const char *s, int32_t length, UErrorCode *status);

#endif  /* __ICU_UCNV_SEL_H__ */
