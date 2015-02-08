

#ifndef __UCSDET_H
#define __UCSDET_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_CONVERSION
#include "unicode/uenum.h"

 

struct UCharsetDetector;
typedef struct UCharsetDetector UCharsetDetector;

struct UCharsetMatch;
typedef struct UCharsetMatch UCharsetMatch;

U_DRAFT UCharsetDetector * U_EXPORT2
ucsdet_open(UErrorCode   *status);

U_DRAFT void U_EXPORT2
ucsdet_close(UCharsetDetector *ucsd);

U_DRAFT void U_EXPORT2
ucsdet_setText(UCharsetDetector *ucsd, const char *textIn, int32_t len, UErrorCode *status);


U_DRAFT void U_EXPORT2
ucsdet_setDeclaredEncoding(UCharsetDetector *ucsd, const char *encoding, int32_t length, UErrorCode *status);


U_DRAFT const UCharsetMatch * U_EXPORT2
ucsdet_detect(UCharsetDetector *ucsd, UErrorCode *status);
    

U_DRAFT const UCharsetMatch ** U_EXPORT2
ucsdet_detectAll(UCharsetDetector *ucsd, int32_t *matchesFound, UErrorCode *status);



U_DRAFT const char * U_EXPORT2
ucsdet_getName(const UCharsetMatch *ucsm, UErrorCode *status);

U_DRAFT int32_t U_EXPORT2
ucsdet_getConfidence(const UCharsetMatch *ucsm, UErrorCode *status);

U_DRAFT const char * U_EXPORT2
ucsdet_getLanguage(const UCharsetMatch *ucsm, UErrorCode *status);


U_DRAFT  int32_t U_EXPORT2
ucsdet_getUChars(const UCharsetMatch *ucsm,
                 UChar *buf, int32_t cap, UErrorCode *status);




U_DRAFT  UEnumeration * U_EXPORT2
ucsdet_getAllDetectableCharsets(const UCharsetDetector *ucsd,  UErrorCode *status);


U_DRAFT  UBool U_EXPORT2
ucsdet_isInputFilterEnabled(const UCharsetDetector *ucsd);


U_DRAFT  UBool U_EXPORT2
ucsdet_enableInputFilter(UCharsetDetector *ucsd, UBool filter);

#endif
#endif   /* __UCSDET_H */


