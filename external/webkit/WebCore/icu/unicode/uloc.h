

#ifndef ULOC_H
#define ULOC_H

#include "unicode/utypes.h"
#include "unicode/uenum.h"


/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_CHINESE            "zh"
/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_ENGLISH            "en"
/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_FRENCH             "fr"
/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_GERMAN             "de"
/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_ITALIAN            "it"
/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_JAPANESE           "ja"
/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_KOREAN             "ko"
/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_SIMPLIFIED_CHINESE "zh_CN"
/** Useful constant for this language. @stable ICU 2.0 */
#define ULOC_TRADITIONAL_CHINESE "zh_TW"

/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_CANADA         "en_CA"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_CANADA_FRENCH  "fr_CA"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_CHINA          "zh_CN"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_PRC            "zh_CN"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_FRANCE         "fr_FR"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_GERMANY        "de_DE"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_ITALY          "it_IT"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_JAPAN          "ja_JP"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_KOREA          "ko_KR"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_TAIWAN         "zh_TW"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_UK             "en_GB"
/** Useful constant for this country/region. @stable ICU 2.0 */
#define ULOC_US             "en_US"

#define ULOC_LANG_CAPACITY 12

#define ULOC_COUNTRY_CAPACITY 4
#define ULOC_FULLNAME_CAPACITY 56


#ifndef U_HIDE_DRAFT_API

#define ULOC_SCRIPT_CAPACITY 6

#define ULOC_KEYWORDS_CAPACITY 50

#define ULOC_KEYWORD_AND_VALUES_CAPACITY 100

#define ULOC_KEYWORD_SEPARATOR '@'
#define ULOC_KEYWORD_ASSIGN '='
#define ULOC_KEYWORD_ITEM_SEPARATOR ';'

#endif /*U_HIDE_DRAFT_API*/

typedef enum {
  /** This is locale the data actually comes from 
   * @stable ICU 2.1
   */
  ULOC_ACTUAL_LOCALE    = 0,
  /** This is the most specific locale supported by ICU 
   * @stable ICU 2.1
   */
  ULOC_VALID_LOCALE    = 1,

#ifndef U_HIDE_DEPRECATED_API
  /** This is the requested locale
   *  @deprecated ICU 2.8 
   */
  ULOC_REQUESTED_LOCALE = 2,
#endif /* U_HIDE_DEPRECATED_API */

  ULOC_DATA_LOCALE_TYPE_LIMIT
} ULocDataLocaleType ;


U_STABLE const char* U_EXPORT2
uloc_getDefault(void);

U_STABLE void U_EXPORT2
uloc_setDefault(const char* localeID,
        UErrorCode*       status);

U_STABLE int32_t U_EXPORT2
uloc_getLanguage(const char*    localeID,
         char* language,
         int32_t languageCapacity,
         UErrorCode* err);

U_DRAFT int32_t U_EXPORT2
uloc_getScript(const char*    localeID,
         char* script,
         int32_t scriptCapacity,
         UErrorCode* err);

U_DRAFT int32_t U_EXPORT2
uloc_getCountry(const char*    localeID,
        char* country,
        int32_t countryCapacity,
        UErrorCode* err);

U_STABLE int32_t U_EXPORT2
uloc_getVariant(const char*    localeID,
        char* variant,
        int32_t variantCapacity,
        UErrorCode* err);


U_STABLE int32_t U_EXPORT2
uloc_getName(const char*    localeID,
         char* name,
         int32_t nameCapacity,
         UErrorCode* err);

U_DRAFT int32_t U_EXPORT2
uloc_canonicalize(const char*    localeID,
         char* name,
         int32_t nameCapacity,
         UErrorCode* err);

U_STABLE const char* U_EXPORT2
uloc_getISO3Language(const char* localeID);


U_STABLE const char* U_EXPORT2
uloc_getISO3Country(const char* localeID);

U_STABLE uint32_t U_EXPORT2
uloc_getLCID(const char* localeID);

U_STABLE int32_t U_EXPORT2
uloc_getDisplayLanguage(const char* locale,
            const char* displayLocale,
            UChar* language,
            int32_t languageCapacity,
            UErrorCode* status);

U_DRAFT int32_t U_EXPORT2
uloc_getDisplayScript(const char* locale,
            const char* displayLocale,
            UChar* script,
            int32_t scriptCapacity,
            UErrorCode* status);

U_STABLE int32_t U_EXPORT2
uloc_getDisplayCountry(const char* locale,
                       const char* displayLocale,
                       UChar* country,
                       int32_t countryCapacity,
                       UErrorCode* status);


U_STABLE int32_t U_EXPORT2
uloc_getDisplayVariant(const char* locale,
                       const char* displayLocale,
                       UChar* variant,
                       int32_t variantCapacity,
                       UErrorCode* status);

U_DRAFT int32_t U_EXPORT2
uloc_getDisplayKeyword(const char* keyword,
                       const char* displayLocale,
                       UChar* dest,
                       int32_t destCapacity,
                       UErrorCode* status);
U_DRAFT int32_t U_EXPORT2
uloc_getDisplayKeywordValue(   const char* locale,
                               const char* keyword,
                               const char* displayLocale,
                               UChar* dest,
                               int32_t destCapacity,
                               UErrorCode* status);
U_STABLE int32_t U_EXPORT2
uloc_getDisplayName(const char* localeID,
            const char* inLocaleID,
            UChar* result,
            int32_t maxResultSize,
            UErrorCode* err);


U_STABLE const char* U_EXPORT2
uloc_getAvailable(int32_t n);

U_STABLE int32_t U_EXPORT2 uloc_countAvailable(void);

U_STABLE const char* const* U_EXPORT2
uloc_getISOLanguages(void);

U_STABLE const char* const* U_EXPORT2
uloc_getISOCountries(void);

U_STABLE int32_t U_EXPORT2
uloc_getParent(const char*    localeID,
                 char* parent,
                 int32_t parentCapacity,
                 UErrorCode* err);




U_DRAFT int32_t U_EXPORT2
uloc_getBaseName(const char*    localeID,
         char* name,
         int32_t nameCapacity,
         UErrorCode* err);

U_DRAFT UEnumeration* U_EXPORT2
uloc_openKeywords(const char* localeID,
                        UErrorCode* status);

U_DRAFT int32_t U_EXPORT2
uloc_getKeywordValue(const char* localeID,
                     const char* keywordName,
                     char* buffer, int32_t bufferCapacity,
                     UErrorCode* status);


U_DRAFT int32_t U_EXPORT2
uloc_setKeywordValue(const char* keywordName,
                     const char* keywordValue,
                     char* buffer, int32_t bufferCapacity,
                     UErrorCode* status);

typedef enum {
  ULOC_ACCEPT_FAILED   = 0,  /* No exact match was found. */
  ULOC_ACCEPT_VALID    = 1,  /* An exact match was found. */
  ULOC_ACCEPT_FALLBACK = 2   /* A fallback was found, for example, 
                                Accept list contained 'ja_JP'
                                which matched available locale 'ja'. */
} UAcceptResult;


U_DRAFT int32_t U_EXPORT2
uloc_acceptLanguageFromHTTP(char *result, int32_t resultAvailable,
                            UAcceptResult *outResult,
                            const char *httpAcceptLanguage,
                            UEnumeration* availableLocales,
                            UErrorCode *status);

U_DRAFT int32_t U_EXPORT2
uloc_acceptLanguage(char *result, int32_t resultAvailable, 
                    UAcceptResult *outResult, const char **acceptList,
                    int32_t acceptListCount,
                    UEnumeration* availableLocales,
                    UErrorCode *status);

/*eof*/


#endif /*_ULOC*/



