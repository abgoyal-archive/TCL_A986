
#ifndef __ICU_UCNV_SEL_H__
#define __ICU_UCNV_SEL_H__

#include "unicode/uset.h"
#include "unicode/utypes.h"
#include "unicode/utf16.h"
#include "unicode/uenum.h"
#include "unicode/ucnv.h"
#include "unicode/localpointer.h"


struct UConverterSelector;
typedef struct UConverterSelector UConverterSelector;
/** @} */

U_STABLE UConverterSelector* U_EXPORT2
ucnvsel_open(const char* const*  converterList, int32_t converterListSize,
             const USet* excludedCodePoints,
             const UConverterUnicodeSet whichSet, UErrorCode* status);

U_STABLE void U_EXPORT2
ucnvsel_close(UConverterSelector *sel);

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

U_DEFINE_LOCAL_OPEN_POINTER(LocalUConverterSelectorPointer, UConverterSelector, ucnvsel_close);

U_NAMESPACE_END

#endif

U_STABLE UConverterSelector* U_EXPORT2
ucnvsel_openFromSerialized(const void* buffer, int32_t length, UErrorCode* status);

U_STABLE int32_t U_EXPORT2
ucnvsel_serialize(const UConverterSelector* sel,
                  void* buffer, int32_t bufferCapacity, UErrorCode* status);

U_STABLE UEnumeration * U_EXPORT2
ucnvsel_selectForString(const UConverterSelector* sel,
                        const UChar *s, int32_t length, UErrorCode *status);

U_STABLE UEnumeration * U_EXPORT2
ucnvsel_selectForUTF8(const UConverterSelector* sel,
                      const char *s, int32_t length, UErrorCode *status);

#endif  /* __ICU_UCNV_SEL_H__ */
