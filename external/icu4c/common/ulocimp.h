
#ifndef ULOCIMP_H
#define ULOCIMP_H

#include "unicode/uloc.h"

U_CAPI UEnumeration* U_EXPORT2
uloc_openKeywordList(const char *keywordList, int32_t keywordListSize, UErrorCode* status);

U_CAPI const UChar * U_EXPORT2
uloc_getTableStringWithFallback(
    const char *path,
    const char *locale,
    const char *tableKey,
    const char *subTableKey,
    const char *itemKey,
    int32_t *pLength,
    UErrorCode *pErrorCode);

/*returns TRUE if a is an ID separator FALSE otherwise*/
#define _isIDSeparator(a) (a == '_' || a == '-')

U_CFUNC const char* 
uloc_getCurrentCountryID(const char* oldID);

U_CFUNC const char* 
uloc_getCurrentLanguageID(const char* oldID);

U_CFUNC int32_t
ulocimp_getLanguage(const char *localeID,
                    char *language, int32_t languageCapacity,
                    const char **pEnd);

U_CFUNC int32_t
ulocimp_getScript(const char *localeID,
                   char *script, int32_t scriptCapacity,
                   const char **pEnd);

U_CFUNC int32_t
ulocimp_getCountry(const char *localeID,
                   char *country, int32_t countryCapacity,
                   const char **pEnd);

#endif
