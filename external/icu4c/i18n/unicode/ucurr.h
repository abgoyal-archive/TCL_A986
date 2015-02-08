#ifndef _UCURR_H_
#define _UCURR_H_

#include "unicode/utypes.h"
#include "unicode/uenum.h"


#if !UCONFIG_NO_FORMATTING


U_STABLE int32_t U_EXPORT2
ucurr_forLocale(const char* locale,
                UChar* buff,
                int32_t buffCapacity,
                UErrorCode* ec);

typedef enum UCurrNameStyle {
    /**
     * Selector for ucurr_getName indicating a symbolic name for a
     * currency, such as "$" for USD.
     * @stable ICU 2.6
     */
    UCURR_SYMBOL_NAME,

    /**
     * Selector for ucurr_getName indicating the long name for a
     * currency, such as "US Dollar" for USD.
     * @stable ICU 2.6
     */
    UCURR_LONG_NAME
} UCurrNameStyle;

#if !UCONFIG_NO_SERVICE
typedef const void* UCurrRegistryKey;

U_STABLE UCurrRegistryKey U_EXPORT2
ucurr_register(const UChar* isoCode, 
                   const char* locale,  
                   UErrorCode* status);
U_STABLE UBool U_EXPORT2
ucurr_unregister(UCurrRegistryKey key, UErrorCode* status);
#endif /* UCONFIG_NO_SERVICE */

U_STABLE const UChar* U_EXPORT2
ucurr_getName(const UChar* currency,
              const char* locale,
              UCurrNameStyle nameStyle,
              UBool* isChoiceFormat,
              int32_t* len,
              UErrorCode* ec);

U_STABLE const UChar* U_EXPORT2
ucurr_getPluralName(const UChar* currency,
                    const char* locale,
                    UBool* isChoiceFormat,
                    const char* pluralCount,
                    int32_t* len,
                    UErrorCode* ec);

U_STABLE int32_t U_EXPORT2
ucurr_getDefaultFractionDigits(const UChar* currency,
                               UErrorCode* ec);

U_STABLE double U_EXPORT2
ucurr_getRoundingIncrement(const UChar* currency,
                           UErrorCode* ec);

typedef enum UCurrCurrencyType {
    /**
     * Select all ISO-4217 currency codes.
     * @stable ICU 3.2
     */
    UCURR_ALL = INT32_MAX,
    /**
     * Select only ISO-4217 commonly used currency codes.
     * These currencies can be found in common use, and they usually have
     * bank notes or coins associated with the currency code.
     * This does not include fund codes, precious metals and other
     * various ISO-4217 codes limited to special financial products.
     * @stable ICU 3.2
     */
    UCURR_COMMON = 1,
    /**
     * Select ISO-4217 uncommon currency codes.
     * These codes respresent fund codes, precious metals and other
     * various ISO-4217 codes limited to special financial products.
     * A fund code is a monetary resource associated with a currency.
     * @stable ICU 3.2
     */
    UCURR_UNCOMMON = 2,
    /**
     * Select only deprecated ISO-4217 codes.
     * These codes are no longer in general public use.
     * @stable ICU 3.2
     */
    UCURR_DEPRECATED = 4,
    /**
     * Select only non-deprecated ISO-4217 codes.
     * These codes are in general public use.
     * @stable ICU 3.2
     */
    UCURR_NON_DEPRECATED = 8
} UCurrCurrencyType;

U_STABLE UEnumeration * U_EXPORT2
ucurr_openISOCurrencies(uint32_t currType, UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
ucurr_countCurrencies(const char* locale, 
                 UDate date, 
                 UErrorCode* ec); 

U_STABLE int32_t U_EXPORT2 
ucurr_forLocaleAndDate(const char* locale, 
                UDate date, 
                int32_t index,
                UChar* buff, 
                int32_t buffCapacity, 
                UErrorCode* ec); 

U_STABLE UEnumeration* U_EXPORT2
ucurr_getKeywordValuesForLocale(const char* key,
                                const char* locale,
                                UBool commonlyUsed,
                                UErrorCode* status);

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
