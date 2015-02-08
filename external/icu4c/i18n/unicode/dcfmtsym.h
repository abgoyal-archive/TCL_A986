
#ifndef DCFMTSYM_H
#define DCFMTSYM_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/uobject.h"
#include "unicode/locid.h"



U_NAMESPACE_BEGIN

class U_I18N_API DecimalFormatSymbols : public UObject {
public:
    /**
     * Constants for specifying a number format symbol.
     * @stable ICU 2.0
     */
    enum ENumberFormatSymbol {
        /** The decimal separator */
        kDecimalSeparatorSymbol,
        /** The grouping separator */
        kGroupingSeparatorSymbol,
        /** The pattern separator */
        kPatternSeparatorSymbol,
        /** The percent sign */
        kPercentSymbol,
        /** Zero*/
        kZeroDigitSymbol,
        /** Character representing a digit in the pattern */
        kDigitSymbol,
        /** The minus sign */
        kMinusSignSymbol,
        /** The plus sign */
        kPlusSignSymbol,
        /** The currency symbol */
        kCurrencySymbol,
        /** The international currency symbol */
        kIntlCurrencySymbol,
        /** The monetary separator */
        kMonetarySeparatorSymbol,
        /** The exponential symbol */
        kExponentialSymbol,
        /** Per mill symbol - replaces kPermillSymbol */
        kPerMillSymbol,
        /** Escape padding character */
        kPadEscapeSymbol,
        /** Infinity symbol */
        kInfinitySymbol,
        /** Nan symbol */
        kNaNSymbol,
        /** Significant digit symbol
         * @stable ICU 3.0 */
        kSignificantDigitSymbol,
        /** The monetary grouping separator
         * @stable ICU 3.6
         */
        kMonetaryGroupingSeparatorSymbol,
        /** count symbol constants */
        kFormatSymbolCount
    };

    /**
      * Constants for specifying currency spacing
      * @draft ICU 4.2
      */
     enum ECurrencySpacing {
       kCurrencyMatch,
       kSurroundingMatch,
       kInsert,
       kCurrencySpacingCount
     };

    /**
     * Create a DecimalFormatSymbols object for the given locale.
     *
     * @param locale    The locale to get symbols for.
     * @param status    Input/output parameter, set to success or
     *                  failure code upon return.
     * @stable ICU 2.0
     */
    DecimalFormatSymbols(const Locale& locale, UErrorCode& status);

    /**
     * Create a DecimalFormatSymbols object for the default locale.
     * This constructor will not fail.  If the resource file data is
     * not available, it will use hard-coded last-resort data and
     * set status to U_USING_FALLBACK_ERROR.
     *
     * @param status    Input/output parameter, set to success or
     *                  failure code upon return.
     * @stable ICU 2.0
     */
    DecimalFormatSymbols( UErrorCode& status);

    // BEGIN android-added: we need a default constructor for performance.
    // Proposed for ICU 4.8: http://icu-project.org/trac/ticket/7392
    DecimalFormatSymbols();
    // END android-added

    /**
     * Copy constructor.
     * @stable ICU 2.0
     */
    DecimalFormatSymbols(const DecimalFormatSymbols&);

    /**
     * Assignment operator.
     * @stable ICU 2.0
     */
    DecimalFormatSymbols& operator=(const DecimalFormatSymbols&);

    /**
     * Destructor.
     * @stable ICU 2.0
     */
    virtual ~DecimalFormatSymbols();

    /**
     * Return true if another object is semantically equal to this one.
     *
     * @param other    the object to be compared with.
     * @return         true if another object is semantically equal to this one.
     * @stable ICU 2.0
     */
    UBool operator==(const DecimalFormatSymbols& other) const;

    /**
     * Return true if another object is semantically unequal to this one.
     *
     * @param other    the object to be compared with.
     * @return         true if another object is semantically unequal to this one.
     * @stable ICU 2.0
     */
    UBool operator!=(const DecimalFormatSymbols& other) const { return !operator==(other); }

    /**
     * Get one of the format symbols by its enum constant.
     * Each symbol is stored as a string so that graphemes
     * (characters with modifier letters) can be used.
     *
     * @param symbol    Constant to indicate a number format symbol.
     * @return    the format symbols by the param 'symbol'
     * @stable ICU 2.0
     */
    inline UnicodeString getSymbol(ENumberFormatSymbol symbol) const;

    /**
     * Set one of the format symbols by its enum constant.
     * Each symbol is stored as a string so that graphemes
     * (characters with modifier letters) can be used.
     *
     * @param symbol    Constant to indicate a number format symbol.
     * @param value     value of the format symbol
     * @stable ICU 2.0
     */
    void setSymbol(ENumberFormatSymbol symbol, const UnicodeString &value);

    /**
     * Returns the locale for which this object was constructed.
     * @stable ICU 2.6
     */
    inline Locale getLocale() const;

    /**
     * Returns the locale for this object. Two flavors are available:
     * valid and actual locale.
     * @stable ICU 2.8
     */
    Locale getLocale(ULocDataLocaleType type, UErrorCode& status) const;

    /**
      * Get pattern string for 'CurrencySpacing' that can be applied to
      * currency format.
      * This API gets the CurrencySpacing data from ResourceBundle. The pattern can
      * be empty if there is no data from current locale and its parent locales.
      *
      * @param type :  kCurrencyMatch, kSurroundingMatch or kInsert.
      * @param beforeCurrency : true if the pattern is for before currency symbol.
      *                         false if the pattern is for after currency symbol.
      * @param status: Input/output parameter, set to success or
      *                  failure code upon return.
      * @return pattern string for currencyMatch, surroundingMatch or spaceInsert.
      *     Return empty string if there is no data for this locale and its parent
      *     locales.
      * @draft ICU 4.2
      */
     const UnicodeString& getPatternForCurrencySpacing(ECurrencySpacing type,
                                                 UBool beforeCurrency,
                                                 UErrorCode& status) const;
     /**
       * Set pattern string for 'CurrencySpacing' that can be applied to
       * currency format.
       *
       * @param type : kCurrencyMatch, kSurroundingMatch or kInsert.
       * @param beforeCurrency : true if the pattern is for before currency symbol.
       *                         false if the pattern is for after currency symbol.
       * @param pattern : pattern string to override current setting.
       * @draft ICU 4.2
       */
     void setPatternForCurrencySpacing(ECurrencySpacing type,
                                       UBool beforeCurrency,
                                       const UnicodeString& pattern);

    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     *
     * @stable ICU 2.2
     */
    virtual UClassID getDynamicClassID() const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     *
     * @stable ICU 2.2
     */
    static UClassID U_EXPORT2 getStaticClassID();

private:
    // BEGIN android-removed: we need a default constructor for performance.
    // DecimalFormatSymbols(); // default constructor not implemented
    // END android-removed

    /**
     * Initializes the symbols from the LocaleElements resource bundle.
     * Note: The organization of LocaleElements badly needs to be
     * cleaned up.
     *
     * @param locale               The locale to get symbols for.
     * @param success              Input/output parameter, set to success or
     *                             failure code upon return.
     * @param useLastResortData    determine if use last resort data
     */
    void initialize(const Locale& locale, UErrorCode& success, UBool useLastResortData = FALSE);

    /**
     * Initialize the symbols from the given array of UnicodeStrings.
     * The array must be of the correct size.
     *
     * @param numberElements    the number format symbols
     * @param numberElementsLength length of numberElements
     */
    void initialize(const UChar** numberElements, int32_t *numberElementsStrLen, int32_t numberElementsLength);

    /**
     * Initialize the symbols with default values.
     */
    void initialize();

    void setCurrencyForSymbols();

public:
    /**
     * _Internal_ function - more efficient version of getSymbol,
     * returning a const reference to one of the symbol strings.
     * The returned reference becomes invalid when the symbol is changed
     * or when the DecimalFormatSymbols are destroyed.
     * ### TODO markus 2002oct11: Consider proposing getConstSymbol() to be really public.
     *
     * @param symbol Constant to indicate a number format symbol.
     * @return the format symbol by the param 'symbol'
     * @internal
     */
    inline const UnicodeString &getConstSymbol(ENumberFormatSymbol symbol) const;

    /**
     * Returns that pattern stored in currecy info. Internal API for use by NumberFormat API.
     * @internal
     */
    inline const UChar* getCurrencyPattern(void) const;

private:
    /**
     * Private symbol strings.
     * They are either loaded from a resource bundle or otherwise owned.
     * setSymbol() clones the symbol string.
     * Readonly aliases can only come from a resource bundle, so that we can always
     * use fastCopyFrom() with them.
     *
     * If DecimalFormatSymbols becomes subclassable and the status of fSymbols changes
     * from private to protected,
     * or when fSymbols can be set any other way that allows them to be readonly aliases
     * to non-resource bundle strings,
     * then regular UnicodeString copies must be used instead of fastCopyFrom().
     *
     * @internal
     */
    UnicodeString fSymbols[kFormatSymbolCount];

    /**
     * Non-symbol variable for getConstSymbol(). Always empty.
     * @internal
     */
    UnicodeString fNoSymbol;

    Locale locale;

    char actualLocale[ULOC_FULLNAME_CAPACITY];
    char validLocale[ULOC_FULLNAME_CAPACITY];
    const UChar* currPattern;

    UnicodeString currencySpcBeforeSym[kCurrencySpacingCount];
    UnicodeString currencySpcAfterSym[kCurrencySpacingCount];
};

// -------------------------------------

inline UnicodeString
DecimalFormatSymbols::getSymbol(ENumberFormatSymbol symbol) const {
    const UnicodeString *strPtr;
    if(symbol < kFormatSymbolCount) {
        strPtr = &fSymbols[symbol];
    } else {
        strPtr = &fNoSymbol;
    }
    return *strPtr;
}

inline const UnicodeString &
DecimalFormatSymbols::getConstSymbol(ENumberFormatSymbol symbol) const {
    const UnicodeString *strPtr;
    if(symbol < kFormatSymbolCount) {
        strPtr = &fSymbols[symbol];
    } else {
        strPtr = &fNoSymbol;
    }
    return *strPtr;
}

// -------------------------------------

inline void
DecimalFormatSymbols::setSymbol(ENumberFormatSymbol symbol, const UnicodeString &value) {
    if(symbol<kFormatSymbolCount) {
        fSymbols[symbol]=value;
    }
}

// -------------------------------------

inline Locale
DecimalFormatSymbols::getLocale() const {
    return locale;
}

inline const UChar*
DecimalFormatSymbols::getCurrencyPattern() const {
    return currPattern;
}
U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _DCFMTSYM
//eof
