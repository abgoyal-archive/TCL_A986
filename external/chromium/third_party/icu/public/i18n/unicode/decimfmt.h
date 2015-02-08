 
#ifndef DECIMFMT_H
#define DECIMFMT_H
 
#include "unicode/utypes.h"
 
#if !UCONFIG_NO_FORMATTING

#include "unicode/dcfmtsym.h"
#include "unicode/numfmt.h"
#include "unicode/locid.h"

union UHashTok;

U_NAMESPACE_BEGIN

U_CDECL_BEGIN

UBool U_CALLCONV decimfmtAffixValueComparator(UHashTok val1, UHashTok val2) ;

UBool U_CALLCONV decimfmtAffixPatternValueComparator(UHashTok val1, UHashTok val2) ;

U_CDECL_END

class DigitList;
class ChoiceFormat;
class CurrencyPluralInfo;
class Hashtable;

class U_I18N_API DecimalFormat: public NumberFormat {
public:
    /**
     * Rounding mode.
     * @stable ICU 2.4
     */
    enum ERoundingMode {
        kRoundCeiling,  /**< Round towards positive infinity */
        kRoundFloor,    /**< Round towards negative infinity */
        kRoundDown,     /**< Round towards zero */
        kRoundUp,       /**< Round away from zero */
        kRoundHalfEven, /**< Round towards the nearest integer, or
                             towards the nearest even integer if equidistant */
        kRoundHalfDown, /**< Round towards the nearest integer, or
                             towards zero if equidistant */
        kRoundHalfUp    /**< Round towards the nearest integer, or
                             away from zero if equidistant */
        // We don't support ROUND_UNNECESSARY
    };

    /**
     * Pad position.
     * @stable ICU 2.4
     */
    enum EPadPosition {
        kPadBeforePrefix,
        kPadAfterPrefix,
        kPadBeforeSuffix,
        kPadAfterSuffix
    };

    /**
     * Create a DecimalFormat using the default pattern and symbols
     * for the default locale. This is a convenient way to obtain a
     * DecimalFormat when internationalization is not the main concern.
     * <P>
     * To obtain standard formats for a given locale, use the factory methods
     * on NumberFormat such as createInstance. These factories will
     * return the most appropriate sub-class of NumberFormat for a given
     * locale.
     * @param status    Output param set to success/failure code. If the
     *                  pattern is invalid this will be set to a failure code.
     * @stable ICU 2.0
     */
    DecimalFormat(UErrorCode& status);

    /**
     * Create a DecimalFormat from the given pattern and the symbols
     * for the default locale. This is a convenient way to obtain a
     * DecimalFormat when internationalization is not the main concern.
     * <P>
     * To obtain standard formats for a given locale, use the factory methods
     * on NumberFormat such as createInstance. These factories will
     * return the most appropriate sub-class of NumberFormat for a given
     * locale.
     * @param pattern   A non-localized pattern string.
     * @param status    Output param set to success/failure code. If the
     *                  pattern is invalid this will be set to a failure code.
     * @stable ICU 2.0
     */
    DecimalFormat(const UnicodeString& pattern,
                  UErrorCode& status);

    /**
     * Create a DecimalFormat from the given pattern and symbols.
     * Use this constructor when you need to completely customize the
     * behavior of the format.
     * <P>
     * To obtain standard formats for a given
     * locale, use the factory methods on NumberFormat such as
     * createInstance or createCurrencyInstance. If you need only minor adjustments
     * to a standard format, you can modify the format returned by
     * a NumberFormat factory method.
     *
     * @param pattern           a non-localized pattern string
     * @param symbolsToAdopt    the set of symbols to be used.  The caller should not
     *                          delete this object after making this call.
     * @param status            Output param set to success/failure code. If the
     *                          pattern is invalid this will be set to a failure code.
     * @stable ICU 2.0
     */
    DecimalFormat(  const UnicodeString& pattern,
                    DecimalFormatSymbols* symbolsToAdopt,
                    UErrorCode& status);

    /**
     * This API is for ICU use only.
     * Create a DecimalFormat from the given pattern, symbols, and style.
     *
     * @param pattern           a non-localized pattern string
     * @param symbolsToAdopt    the set of symbols to be used.  The caller should not
     *                          delete this object after making this call.
     * @param style             style of decimal format, kNumberStyle etc.
     * @param status            Output param set to success/failure code. If the
     *                          pattern is invalid this will be set to a failure code.
     * @internal ICU 4.2
     */
    DecimalFormat(  const UnicodeString& pattern,
                    DecimalFormatSymbols* symbolsToAdopt,
                    NumberFormat::EStyles style,
                    UErrorCode& status);

    /**
     * Create a DecimalFormat from the given pattern and symbols.
     * Use this constructor when you need to completely customize the
     * behavior of the format.
     * <P>
     * To obtain standard formats for a given
     * locale, use the factory methods on NumberFormat such as
     * createInstance or createCurrencyInstance. If you need only minor adjustments
     * to a standard format, you can modify the format returned by
     * a NumberFormat factory method.
     *
     * @param pattern           a non-localized pattern string
     * @param symbolsToAdopt    the set of symbols to be used.  The caller should not
     *                          delete this object after making this call.
     * @param parseError        Output param to receive errors occured during parsing 
     * @param status            Output param set to success/failure code. If the
     *                          pattern is invalid this will be set to a failure code.
     * @stable ICU 2.0
     */
    DecimalFormat(  const UnicodeString& pattern,
                    DecimalFormatSymbols* symbolsToAdopt,
                    UParseError& parseError,
                    UErrorCode& status);
    /**
     * Create a DecimalFormat from the given pattern and symbols.
     * Use this constructor when you need to completely customize the
     * behavior of the format.
     * <P>
     * To obtain standard formats for a given
     * locale, use the factory methods on NumberFormat such as
     * createInstance or createCurrencyInstance. If you need only minor adjustments
     * to a standard format, you can modify the format returned by
     * a NumberFormat factory method.
     *
     * @param pattern           a non-localized pattern string
     * @param symbols   the set of symbols to be used
     * @param status            Output param set to success/failure code. If the
     *                          pattern is invalid this will be set to a failure code.
     * @stable ICU 2.0
     */
    DecimalFormat(  const UnicodeString& pattern,
                    const DecimalFormatSymbols& symbols,
                    UErrorCode& status);

    /**
     * Copy constructor.
     * 
     * @param source    the DecimalFormat object to be copied from.
     * @stable ICU 2.0
     */
    DecimalFormat(const DecimalFormat& source);

    /**
     * Assignment operator.
     *
     * @param rhs    the DecimalFormat object to be copied.
     * @stable ICU 2.0
     */
    DecimalFormat& operator=(const DecimalFormat& rhs);

    /**
     * Destructor.
     * @stable ICU 2.0
     */
    virtual ~DecimalFormat();

    /**
     * Clone this Format object polymorphically. The caller owns the
     * result and should delete it when done.
     *
     * @return    a polymorphic copy of this DecimalFormat.
     * @stable ICU 2.0
     */
    virtual Format* clone(void) const;

    /**
     * Return true if the given Format objects are semantically equal.
     * Objects of different subclasses are considered unequal.
     *
     * @param other    the object to be compared with.
     * @return         true if the given Format objects are semantically equal.
     * @stable ICU 2.0
     */
    virtual UBool operator==(const Format& other) const;

    /**
     * Format a double or long number using base-10 representation.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param pos       On input: an alignment field, if desired.
     *                  On output: the offsets of the alignment field.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
    */
    virtual UnicodeString& format(double number,
                                  UnicodeString& appendTo,
                                  FieldPosition& pos) const;
    /**
     * Format a long number using base-10 representation.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param pos       On input: an alignment field, if desired.
     *                  On output: the offsets of the alignment field.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    virtual UnicodeString& format(int32_t number,
                                  UnicodeString& appendTo,
                                  FieldPosition& pos) const;
    /**
     * Format an int64 number using base-10 representation.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param pos       On input: an alignment field, if desired.
     *                  On output: the offsets of the alignment field.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.8
     */
    virtual UnicodeString& format(int64_t number,
                                  UnicodeString& appendTo,
                                  FieldPosition& pos) const;

    /**
     * Format a Formattable using base-10 representation.
     *
     * @param obj       The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param pos       On input: an alignment field, if desired.
     *                  On output: the offsets of the alignment field.
     * @param status    Error code indicating success or failure.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    virtual UnicodeString& format(const Formattable& obj,
                                  UnicodeString& appendTo,
                                  FieldPosition& pos,
                                  UErrorCode& status) const;

    /**
     * Redeclared NumberFormat method.
     * Formats an object to produce a string.
     *
     * @param obj       The object to format.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param status    Output parameter filled in with success or failure status.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    UnicodeString& format(const Formattable& obj,
                          UnicodeString& appendTo,
                          UErrorCode& status) const;

    /**
     * Redeclared NumberFormat method.
     * Format a double number.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    UnicodeString& format(double number,
                          UnicodeString& appendTo) const;

    /**
     * Redeclared NumberFormat method.
     * Format a long number. These methods call the NumberFormat
     * pure virtual format() methods with the default FieldPosition.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    UnicodeString& format(int32_t number,
                          UnicodeString& appendTo) const;

    /**
     * Redeclared NumberFormat method.
     * Format an int64 number. These methods call the NumberFormat
     * pure virtual format() methods with the default FieldPosition.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.8
     */
    UnicodeString& format(int64_t number,
                          UnicodeString& appendTo) const;
   /**
    * Parse the given string using this object's choices. The method
    * does string comparisons to try to find an optimal match.
    * If no object can be parsed, index is unchanged, and NULL is
    * returned.  The result is returned as the most parsimonious
    * type of Formattable that will accomodate all of the
    * necessary precision.  For example, if the result is exactly 12,
    * it will be returned as a long.  However, if it is 1.5, it will
    * be returned as a double.
    *
    * @param text           The text to be parsed.
    * @param result         Formattable to be set to the parse result.
    *                       If parse fails, return contents are undefined.
    * @param parsePosition  The position to start parsing at on input.
    *                       On output, moved to after the last successfully
    *                       parse character. On parse failure, does not change.
    * @see Formattable
    * @stable ICU 2.0
    */
    virtual void parse(const UnicodeString& text,
                       Formattable& result,
                       ParsePosition& parsePosition) const;

    // Declare here again to get rid of function hiding problems.
    /** 
     * Parse the given string using this object's choices.
     *
     * @param text           The text to be parsed.
     * @param result         Formattable to be set to the parse result.
     * @param status    Output parameter filled in with success or failure status.
     * @stable ICU 2.0
     */
    virtual void parse(const UnicodeString& text, 
                       Formattable& result, 
                       UErrorCode& status) const;

    /**
     * Parses text from the given string as a currency amount.  Unlike
     * the parse() method, this method will attempt to parse a generic
     * currency name, searching for a match of this object's locale's
     * currency display names, or for a 3-letter ISO currency code.
     * This method will fail if this format is not a currency format,
     * that is, if it does not contain the currency pattern symbol
     * (U+00A4) in its prefix or suffix.
     *
     * @param text the string to parse
     * @param result output parameter to receive result. This will have
     * its currency set to the parsed ISO currency code.
     * @param pos input-output position; on input, the position within
     * text to match; must have 0 <= pos.getIndex() < text.length();
     * on output, the position after the last matched character. If
     * the parse fails, the position in unchanged upon output.
     * @return a reference to result
     * @internal
     */
    virtual Formattable& parseCurrency(const UnicodeString& text,
                                       Formattable& result,
                                       ParsePosition& pos) const;

    /**
     * Returns the decimal format symbols, which is generally not changed
     * by the programmer or user.
     * @return desired DecimalFormatSymbols
     * @see DecimalFormatSymbols
     * @stable ICU 2.0
     */
    virtual const DecimalFormatSymbols* getDecimalFormatSymbols(void) const;

    /**
     * Sets the decimal format symbols, which is generally not changed
     * by the programmer or user.
     * @param symbolsToAdopt DecimalFormatSymbols to be adopted.
     * @stable ICU 2.0
     */
    virtual void adoptDecimalFormatSymbols(DecimalFormatSymbols* symbolsToAdopt);

    /**
     * Sets the decimal format symbols, which is generally not changed
     * by the programmer or user.
     * @param symbols DecimalFormatSymbols.
     * @stable ICU 2.0
     */
    virtual void setDecimalFormatSymbols(const DecimalFormatSymbols& symbols);


    /**
     * Returns the currency plural format information, 
     * which is generally not changed by the programmer or user.
     * @return desired CurrencyPluralInfo
     * @draft ICU 4.2
     */
    virtual const CurrencyPluralInfo* getCurrencyPluralInfo(void) const;

    /**
     * Sets the currency plural format information, 
     * which is generally not changed by the programmer or user.
     * @param toAdopt CurrencyPluralInfo to be adopted.
     * @draft ICU 4.2
     */
    virtual void adoptCurrencyPluralInfo(CurrencyPluralInfo* toAdopt);

    /**
     * Sets the currency plural format information, 
     * which is generally not changed by the programmer or user.
     * @param info Currency Plural Info.
     * @draft ICU 4.2
     */
    virtual void setCurrencyPluralInfo(const CurrencyPluralInfo& info);


    /**
     * Get the positive prefix.
     *
     * @param result    Output param which will receive the positive prefix.
     * @return          A reference to 'result'.
     * Examples: +123, $123, sFr123
     * @stable ICU 2.0
     */
    UnicodeString& getPositivePrefix(UnicodeString& result) const;

    /**
     * Set the positive prefix.
     *
     * @param newValue    the new value of the the positive prefix to be set.
     * Examples: +123, $123, sFr123
     * @stable ICU 2.0
     */
    virtual void setPositivePrefix(const UnicodeString& newValue);

    /**
     * Get the negative prefix.
     *
     * @param result    Output param which will receive the negative prefix.
     * @return          A reference to 'result'.
     * Examples: -123, ($123) (with negative suffix), sFr-123
     * @stable ICU 2.0
     */
    UnicodeString& getNegativePrefix(UnicodeString& result) const;

    /**
     * Set the negative prefix.
     *
     * @param newValue    the new value of the the negative prefix to be set.
     * Examples: -123, ($123) (with negative suffix), sFr-123
     * @stable ICU 2.0
     */
    virtual void setNegativePrefix(const UnicodeString& newValue);

    /**
     * Get the positive suffix.
     *
     * @param result    Output param which will receive the positive suffix.
     * @return          A reference to 'result'.
     * Example: 123%
     * @stable ICU 2.0
     */
    UnicodeString& getPositiveSuffix(UnicodeString& result) const;

    /**
     * Set the positive suffix.
     *
     * @param newValue    the new value of the positive suffix to be set.
     * Example: 123%
     * @stable ICU 2.0
     */
    virtual void setPositiveSuffix(const UnicodeString& newValue);

    /**
     * Get the negative suffix.
     *
     * @param result    Output param which will receive the negative suffix.
     * @return          A reference to 'result'.
     * Examples: -123%, ($123) (with positive suffixes)
     * @stable ICU 2.0
     */
    UnicodeString& getNegativeSuffix(UnicodeString& result) const;

    /**
     * Set the negative suffix.
     *
     * @param newValue    the new value of the negative suffix to be set.
     * Examples: 123%
     * @stable ICU 2.0
     */
    virtual void setNegativeSuffix(const UnicodeString& newValue);

    /**
     * Get the multiplier for use in percent, permill, etc.
     * For a percentage, set the suffixes to have "%" and the multiplier to be 100.
     * (For Arabic, use arabic percent symbol).
     * For a permill, set the suffixes to have "\\u2031" and the multiplier to be 1000.
     *
     * @return    the multiplier for use in percent, permill, etc.
     * Examples: with 100, 1.23 -> "123", and "123" -> 1.23
     * @stable ICU 2.0
     */
    int32_t getMultiplier(void) const;

    /**
     * Set the multiplier for use in percent, permill, etc.
     * For a percentage, set the suffixes to have "%" and the multiplier to be 100.
     * (For Arabic, use arabic percent symbol).
     * For a permill, set the suffixes to have "\\u2031" and the multiplier to be 1000.
     *
     * @param newValue    the new value of the multiplier for use in percent, permill, etc.
     * Examples: with 100, 1.23 -> "123", and "123" -> 1.23
     * @stable ICU 2.0
     */
    virtual void setMultiplier(int32_t newValue);

    /**
     * Get the rounding increment.
     * @return A positive rounding increment, or 0.0 if rounding
     * is not in effect.
     * @see #setRoundingIncrement
     * @see #getRoundingMode
     * @see #setRoundingMode
     * @stable ICU 2.0
     */
    virtual double getRoundingIncrement(void) const;

    /**
     * Set the rounding increment.  This method also controls whether
     * rounding is enabled.
     * @param newValue A positive rounding increment, or 0.0 to disable rounding.
     * Negative increments are equivalent to 0.0.
     * @see #getRoundingIncrement
     * @see #getRoundingMode
     * @see #setRoundingMode
     * @stable ICU 2.0
     */
    virtual void setRoundingIncrement(double newValue);

    /**
     * Get the rounding mode.
     * @return A rounding mode
     * @see #setRoundingIncrement
     * @see #getRoundingIncrement
     * @see #setRoundingMode
     * @stable ICU 2.0
     */
    virtual ERoundingMode getRoundingMode(void) const;

    /**
     * Set the rounding mode.  This has no effect unless the rounding
     * increment is greater than zero.
     * @param roundingMode A rounding mode
     * @see #setRoundingIncrement
     * @see #getRoundingIncrement
     * @see #getRoundingMode
     * @stable ICU 2.0
     */
    virtual void setRoundingMode(ERoundingMode roundingMode);

    /**
     * Get the width to which the output of format() is padded.
     * The width is counted in 16-bit code units.
     * @return the format width, or zero if no padding is in effect
     * @see #setFormatWidth
     * @see #getPadCharacterString
     * @see #setPadCharacter
     * @see #getPadPosition
     * @see #setPadPosition
     * @stable ICU 2.0
     */
    virtual int32_t getFormatWidth(void) const;

    /**
     * Set the width to which the output of format() is padded.
     * The width is counted in 16-bit code units.
     * This method also controls whether padding is enabled.
     * @param width the width to which to pad the result of
     * format(), or zero to disable padding.  A negative
     * width is equivalent to 0.
     * @see #getFormatWidth
     * @see #getPadCharacterString
     * @see #setPadCharacter
     * @see #getPadPosition
     * @see #setPadPosition
     * @stable ICU 2.0
     */
    virtual void setFormatWidth(int32_t width);

    /**
     * Get the pad character used to pad to the format width.  The
     * default is ' '.
     * @return a string containing the pad character. This will always
     * have a length of one 32-bit code point.
     * @see #setFormatWidth
     * @see #getFormatWidth
     * @see #setPadCharacter
     * @see #getPadPosition
     * @see #setPadPosition
     * @stable ICU 2.0
     */
    virtual UnicodeString getPadCharacterString() const;

    /**
     * Set the character used to pad to the format width.  If padding
     * is not enabled, then this will take effect if padding is later
     * enabled.
     * @param padChar a string containing the pad charcter. If the string
     * has length 0, then the pad characer is set to ' '.  Otherwise
     * padChar.char32At(0) will be used as the pad character.
     * @see #setFormatWidth
     * @see #getFormatWidth
     * @see #getPadCharacterString
     * @see #getPadPosition
     * @see #setPadPosition
     * @stable ICU 2.0
     */
    virtual void setPadCharacter(const UnicodeString &padChar);

    /**
     * Get the position at which padding will take place.  This is the location
     * at which padding will be inserted if the result of format()
     * is shorter than the format width.
     * @return the pad position, one of kPadBeforePrefix,
     * kPadAfterPrefix, kPadBeforeSuffix, or
     * kPadAfterSuffix.
     * @see #setFormatWidth
     * @see #getFormatWidth
     * @see #setPadCharacter
     * @see #getPadCharacterString
     * @see #setPadPosition
     * @see #EPadPosition
     * @stable ICU 2.0
     */
    virtual EPadPosition getPadPosition(void) const;

    /**
     * Set the position at which padding will take place.  This is the location
     * at which padding will be inserted if the result of format()
     * is shorter than the format width.  This has no effect unless padding is
     * enabled.
     * @param padPos the pad position, one of kPadBeforePrefix,
     * kPadAfterPrefix, kPadBeforeSuffix, or
     * kPadAfterSuffix.
     * @see #setFormatWidth
     * @see #getFormatWidth
     * @see #setPadCharacter
     * @see #getPadCharacterString
     * @see #getPadPosition
     * @see #EPadPosition
     * @stable ICU 2.0
     */
    virtual void setPadPosition(EPadPosition padPos);

    /**
     * Return whether or not scientific notation is used.
     * @return TRUE if this object formats and parses scientific notation
     * @see #setScientificNotation
     * @see #getMinimumExponentDigits
     * @see #setMinimumExponentDigits
     * @see #isExponentSignAlwaysShown
     * @see #setExponentSignAlwaysShown
     * @stable ICU 2.0
     */
    virtual UBool isScientificNotation(void);

    /**
     * Set whether or not scientific notation is used. When scientific notation
     * is used, the effective maximum number of integer digits is <= 8.  If the
     * maximum number of integer digits is set to more than 8, the effective
     * maximum will be 1.  This allows this call to generate a 'default' scientific
     * number format without additional changes.
     * @param useScientific TRUE if this object formats and parses scientific
     * notation
     * @see #isScientificNotation
     * @see #getMinimumExponentDigits
     * @see #setMinimumExponentDigits
     * @see #isExponentSignAlwaysShown
     * @see #setExponentSignAlwaysShown
     * @stable ICU 2.0
     */
    virtual void setScientificNotation(UBool useScientific);

    /**
     * Return the minimum exponent digits that will be shown.
     * @return the minimum exponent digits that will be shown
     * @see #setScientificNotation
     * @see #isScientificNotation
     * @see #setMinimumExponentDigits
     * @see #isExponentSignAlwaysShown
     * @see #setExponentSignAlwaysShown
     * @stable ICU 2.0
     */
    virtual int8_t getMinimumExponentDigits(void) const;

    /**
     * Set the minimum exponent digits that will be shown.  This has no
     * effect unless scientific notation is in use.
     * @param minExpDig a value >= 1 indicating the fewest exponent digits
     * that will be shown.  Values less than 1 will be treated as 1.
     * @see #setScientificNotation
     * @see #isScientificNotation
     * @see #getMinimumExponentDigits
     * @see #isExponentSignAlwaysShown
     * @see #setExponentSignAlwaysShown
     * @stable ICU 2.0
     */
    virtual void setMinimumExponentDigits(int8_t minExpDig);

    /**
     * Return whether the exponent sign is always shown.
     * @return TRUE if the exponent is always prefixed with either the
     * localized minus sign or the localized plus sign, false if only negative
     * exponents are prefixed with the localized minus sign.
     * @see #setScientificNotation
     * @see #isScientificNotation
     * @see #setMinimumExponentDigits
     * @see #getMinimumExponentDigits
     * @see #setExponentSignAlwaysShown
     * @stable ICU 2.0
     */
    virtual UBool isExponentSignAlwaysShown(void);

    /**
     * Set whether the exponent sign is always shown.  This has no effect
     * unless scientific notation is in use.
     * @param expSignAlways TRUE if the exponent is always prefixed with either
     * the localized minus sign or the localized plus sign, false if only
     * negative exponents are prefixed with the localized minus sign.
     * @see #setScientificNotation
     * @see #isScientificNotation
     * @see #setMinimumExponentDigits
     * @see #getMinimumExponentDigits
     * @see #isExponentSignAlwaysShown
     * @stable ICU 2.0
     */
    virtual void setExponentSignAlwaysShown(UBool expSignAlways);

    /**
     * Return the grouping size. Grouping size is the number of digits between
     * grouping separators in the integer portion of a number.  For example,
     * in the number "123,456.78", the grouping size is 3.
     *
     * @return    the grouping size.
     * @see setGroupingSize
     * @see NumberFormat::isGroupingUsed
     * @see DecimalFormatSymbols::getGroupingSeparator
     * @stable ICU 2.0
     */
    int32_t getGroupingSize(void) const;

    /**
     * Set the grouping size. Grouping size is the number of digits between
     * grouping separators in the integer portion of a number.  For example,
     * in the number "123,456.78", the grouping size is 3.
     *
     * @param newValue    the new value of the grouping size.
     * @see getGroupingSize
     * @see NumberFormat::setGroupingUsed
     * @see DecimalFormatSymbols::setGroupingSeparator
     * @stable ICU 2.0
     */
    virtual void setGroupingSize(int32_t newValue);

    /**
     * Return the secondary grouping size. In some locales one
     * grouping interval is used for the least significant integer
     * digits (the primary grouping size), and another is used for all
     * others (the secondary grouping size).  A formatter supporting a
     * secondary grouping size will return a positive integer unequal
     * to the primary grouping size returned by
     * getGroupingSize().  For example, if the primary
     * grouping size is 4, and the secondary grouping size is 2, then
     * the number 123456789 formats as "1,23,45,6789", and the pattern
     * appears as "#,##,###0".
     * @return the secondary grouping size, or a value less than
     * one if there is none
     * @see setSecondaryGroupingSize
     * @see NumberFormat::isGroupingUsed
     * @see DecimalFormatSymbols::getGroupingSeparator
     * @stable ICU 2.4
     */
    int32_t getSecondaryGroupingSize(void) const;

    /**
     * Set the secondary grouping size. If set to a value less than 1,
     * then secondary grouping is turned off, and the primary grouping
     * size is used for all intervals, not just the least significant.
     *
     * @param newValue    the new value of the secondary grouping size.
     * @see getSecondaryGroupingSize
     * @see NumberFormat#setGroupingUsed
     * @see DecimalFormatSymbols::setGroupingSeparator
     * @stable ICU 2.4
     */
    virtual void setSecondaryGroupingSize(int32_t newValue);

    /**
     * Allows you to get the behavior of the decimal separator with integers.
     * (The decimal separator will always appear with decimals.)
     *
     * @return    TRUE if the decimal separator always appear with decimals.
     * Example: Decimal ON: 12345 -> 12345.; OFF: 12345 -> 12345
     * @stable ICU 2.0
     */
    UBool isDecimalSeparatorAlwaysShown(void) const;

    /**
     * Allows you to set the behavior of the decimal separator with integers.
     * (The decimal separator will always appear with decimals.)
     *
     * @param newValue    set TRUE if the decimal separator will always appear with decimals.
     * Example: Decimal ON: 12345 -> 12345.; OFF: 12345 -> 12345
     * @stable ICU 2.0
     */
    virtual void setDecimalSeparatorAlwaysShown(UBool newValue);

    /**
     * Synthesizes a pattern string that represents the current state
     * of this Format object.
     *
     * @param result    Output param which will receive the pattern.
     *                  Previous contents are deleted.
     * @return          A reference to 'result'.
     * @see applyPattern
     * @stable ICU 2.0
     */
    virtual UnicodeString& toPattern(UnicodeString& result) const;

    /**
     * Synthesizes a localized pattern string that represents the current
     * state of this Format object.
     *
     * @param result    Output param which will receive the localized pattern.
     *                  Previous contents are deleted.
     * @return          A reference to 'result'.
     * @see applyPattern
     * @stable ICU 2.0
     */
    virtual UnicodeString& toLocalizedPattern(UnicodeString& result) const;
 
    /**
     * Apply the given pattern to this Format object.  A pattern is a
     * short-hand specification for the various formatting properties.
     * These properties can also be changed individually through the
     * various setter methods.
     * <P>
     * There is no limit to integer digits are set
     * by this routine, since that is the typical end-user desire;
     * use setMaximumInteger if you want to set a real value.
     * For negative numbers, use a second pattern, separated by a semicolon
     * <pre>
     * .      Example "#,#00.0#" -> 1,234.56
     * </pre>
     * This means a minimum of 2 integer digits, 1 fraction digit, and
     * a maximum of 2 fraction digits.
     * <pre>
     * .      Example: "#,#00.0#;(#,#00.0#)" for negatives in parantheses.
     * </pre>
     * In negative patterns, the minimum and maximum counts are ignored;
     * these are presumed to be set in the positive pattern.
     *
     * @param pattern    The pattern to be applied.
     * @param parseError Struct to recieve information on position 
     *                   of error if an error is encountered
     * @param status     Output param set to success/failure code on
     *                   exit. If the pattern is invalid, this will be
     *                   set to a failure result.
     * @stable ICU 2.0
     */
    virtual void applyPattern(const UnicodeString& pattern,
                             UParseError& parseError,
                             UErrorCode& status);
    /**
     * Sets the pattern.
     * @param pattern   The pattern to be applied.
     * @param status    Output param set to success/failure code on
     *                  exit. If the pattern is invalid, this will be
     *                  set to a failure result.
     * @stable ICU 2.0
     */  
    virtual void applyPattern(const UnicodeString& pattern,
                             UErrorCode& status);

    /**
     * Apply the given pattern to this Format object.  The pattern
     * is assumed to be in a localized notation. A pattern is a
     * short-hand specification for the various formatting properties.
     * These properties can also be changed individually through the
     * various setter methods.
     * <P>
     * There is no limit to integer digits are set
     * by this routine, since that is the typical end-user desire;
     * use setMaximumInteger if you want to set a real value.
     * For negative numbers, use a second pattern, separated by a semicolon
     * <pre>
     * .      Example "#,#00.0#" -> 1,234.56
     * </pre>
     * This means a minimum of 2 integer digits, 1 fraction digit, and
     * a maximum of 2 fraction digits.
     *
     * Example: "#,#00.0#;(#,#00.0#)" for negatives in parantheses.
     *
     * In negative patterns, the minimum and maximum counts are ignored;
     * these are presumed to be set in the positive pattern.
     *
     * @param pattern   The localized pattern to be applied.
     * @param parseError Struct to recieve information on position 
     *                   of error if an error is encountered
     * @param status    Output param set to success/failure code on
     *                  exit. If the pattern is invalid, this will be
     *                  set to a failure result.
     * @stable ICU 2.0
     */
    virtual void applyLocalizedPattern(const UnicodeString& pattern,
                                       UParseError& parseError,
                                       UErrorCode& status);

    /**
     * Apply the given pattern to this Format object.
     *
     * @param pattern   The localized pattern to be applied.
     * @param status    Output param set to success/failure code on
     *                  exit. If the pattern is invalid, this will be
     *                  set to a failure result.
     * @stable ICU 2.0
     */
    virtual void applyLocalizedPattern(const UnicodeString& pattern,
                                       UErrorCode& status);


    /**
     * Sets the maximum number of digits allowed in the integer portion of a
     * number. This override limits the integer digit count to 309.
     *
     * @param newValue    the new value of the maximum number of digits 
     *                      allowed in the integer portion of a number.
     * @see NumberFormat#setMaximumIntegerDigits
     * @stable ICU 2.0
     */
    virtual void setMaximumIntegerDigits(int32_t newValue);

    /**
     * Sets the minimum number of digits allowed in the integer portion of a
     * number. This override limits the integer digit count to 309.
     * 
     * @param newValue    the new value of the minimum number of digits 
     *                      allowed in the integer portion of a number.
     * @see NumberFormat#setMinimumIntegerDigits
     * @stable ICU 2.0
     */
    virtual void setMinimumIntegerDigits(int32_t newValue);

    /**
     * Sets the maximum number of digits allowed in the fraction portion of a
     * number. This override limits the fraction digit count to 340.
     *
     * @param newValue    the new value of the maximum number of digits 
     *                    allowed in the fraction portion of a number.
     * @see NumberFormat#setMaximumFractionDigits
     * @stable ICU 2.0
     */
    virtual void setMaximumFractionDigits(int32_t newValue);

    /**
     * Sets the minimum number of digits allowed in the fraction portion of a
     * number. This override limits the fraction digit count to 340.
     *
     * @param newValue    the new value of the minimum number of digits 
     *                    allowed in the fraction portion of a number.
     * @see NumberFormat#setMinimumFractionDigits
     * @stable ICU 2.0
     */
    virtual void setMinimumFractionDigits(int32_t newValue);

    /**
     * Returns the minimum number of significant digits that will be
     * displayed. This value has no effect unless areSignificantDigitsUsed()
     * returns true.
     * @return the fewest significant digits that will be shown
     * @stable ICU 3.0
     */
    int32_t getMinimumSignificantDigits() const;

    /**
     * Returns the maximum number of significant digits that will be
     * displayed. This value has no effect unless areSignificantDigitsUsed()
     * returns true.
     * @return the most significant digits that will be shown
     * @stable ICU 3.0
     */
    int32_t getMaximumSignificantDigits() const;

    /**
     * Sets the minimum number of significant digits that will be
     * displayed.  If <code>min</code> is less than one then it is set
     * to one.  If the maximum significant digits count is less than
     * <code>min</code>, then it is set to <code>min</code>. This
     * value has no effect unless areSignificantDigits() returns true.
     * @param min the fewest significant digits to be shown 
     * @stable ICU 3.0
     */
    void setMinimumSignificantDigits(int32_t min);

    /**
     * Sets the maximum number of significant digits that will be
     * displayed.  If <code>max</code> is less than one then it is set
     * to one.  If the minimum significant digits count is greater
     * than <code>max</code>, then it is set to <code>max</code>.
     * This value has no effect unless areSignificantDigits() returns
     * true.
     * @param max the most significant digits to be shown 
     * @stable ICU 3.0
     */
    void setMaximumSignificantDigits(int32_t max);

    /**
     * Returns true if significant digits are in use, or false if
     * integer and fraction digit counts are in use.
     * @return true if significant digits are in use
     * @stable ICU 3.0
     */
    UBool areSignificantDigitsUsed() const;

    /**
     * Sets whether significant digits are in use, or integer and
     * fraction digit counts are in use.
     * @param useSignificantDigits true to use significant digits, or
     * false to use integer and fraction digit counts
     * @stable ICU 3.0
     */
    void setSignificantDigitsUsed(UBool useSignificantDigits);

 public:
    /**
     * Sets the currency used to display currency
     * amounts.  This takes effect immediately, if this format is a
     * currency format.  If this format is not a currency format, then
     * the currency is used if and when this object becomes a
     * currency format through the application of a new pattern.
     * @param theCurrency a 3-letter ISO code indicating new currency
     * to use.  It need not be null-terminated.  May be the empty
     * string or NULL to indicate no currency.
     * @param ec input-output error code
     * @stable ICU 3.0
     */
    virtual void setCurrency(const UChar* theCurrency, UErrorCode& ec);

    /**
     * Sets the currency used to display currency amounts.  See
     * setCurrency(const UChar*, UErrorCode&).
     * @deprecated ICU 3.0. Use setCurrency(const UChar*, UErrorCode&).
     */
    virtual void setCurrency(const UChar* theCurrency);

    /**
     * The resource tags we use to retrieve decimal format data from
     * locale resource bundles.
     * @deprecated ICU 3.4. This string has no public purpose. Please don't use it.
     */
    static const char fgNumberPatterns[];

public:

    /**
     * Return the class ID for this class.  This is useful only for
     * comparing to a return value from getDynamicClassID().  For example:
     * <pre>
     * .      Base* polymorphic_pointer = createPolymorphicObject();
     * .      if (polymorphic_pointer->getDynamicClassID() ==
     * .          Derived::getStaticClassID()) ...
     * </pre>
     * @return          The class ID for all objects of this class.
     * @stable ICU 2.0
     */
    static UClassID U_EXPORT2 getStaticClassID(void);

    /**
     * Returns a unique class ID POLYMORPHICALLY.  Pure virtual override.
     * This method is to implement a simple version of RTTI, since not all
     * C++ compilers support genuine RTTI.  Polymorphic operator==() and
     * clone() methods call this method.
     *
     * @return          The class ID for this object. All objects of a
     *                  given class have the same class ID.  Objects of
     *                  other classes have different class IDs.
     * @stable ICU 2.0
     */
    virtual UClassID getDynamicClassID(void) const;

private:
    friend UBool U_CALLCONV decimfmtAffixValueComparator(UHashTok val1, UHashTok val2);
    friend UBool U_CALLCONV decimfmtAffixPatternValueComparator(UHashTok val1, UHashTok val2);

    DecimalFormat(); // default constructor not implemented

    int32_t precision(UBool isIntegral) const;

    /**
     * Do real work of constructing a new DecimalFormat.
     */
    void construct(UErrorCode&               status,
                   UParseError&             parseErr,
                   const UnicodeString*     pattern = 0,
                   DecimalFormatSymbols*    symbolsToAdopt = 0
                   );

    /**
     * Does the real work of generating a pattern.
     *
     * @param result     Output param which will receive the pattern.
     *                   Previous contents are deleted.
     * @param localized  TRUE return localized pattern.
     * @return           A reference to 'result'.
     */
    UnicodeString& toPattern(UnicodeString& result, UBool localized) const;

    /**
     * Does the real work of applying a pattern.
     * @param pattern    The pattern to be applied.
     * @param localized  If true, the pattern is localized; else false.
     * @param parseError Struct to recieve information on position 
     *                   of error if an error is encountered
     * @param status     Output param set to success/failure code on
     *                   exit. If the pattern is invalid, this will be
     *                   set to a failure result.
     */
    void applyPattern(const UnicodeString& pattern,
                            UBool localized,
                            UParseError& parseError,
                            UErrorCode& status);

    /* 
     * similar to applyPattern, but without re-gen affix for currency 
     */
    void applyPatternInternally(const UnicodeString& pluralCount,
                                const UnicodeString& pattern,
                                UBool localized,
                                UParseError& parseError,
                                UErrorCode& status);

    /*
     * only apply pattern without expand affixes
     */
    void applyPatternWithoutExpandAffix(const UnicodeString& pattern,
                                        UBool localized,
                                        UParseError& parseError,
                                        UErrorCode& status);


    /*
     * expand affixes (after apply patter) and re-compute fFormatWidth
     */
    void expandAffixAdjustWidth(const UnicodeString* pluralCount);

    
    /**
     * Do the work of formatting a number, either a double or a long.
     *
     * @param appendTo       Output parameter to receive result.
     *                       Result is appended to existing contents.
     * @param fieldPosition  On input: an alignment field, if desired.
     *                       On output: the offsets of the alignment field.
     * @param digits         the digits to be formatted.
     * @param isInteger      if TRUE format the digits as Integer.
     * @return               Reference to 'appendTo' parameter.
     */
    UnicodeString& subformat(UnicodeString& appendTo,
                             FieldPosition& fieldPosition,
                             DigitList& digits,
                             UBool         isInteger) const;


    void parse(const UnicodeString& text,
               Formattable& result,
               ParsePosition& pos,
               UBool parseCurrency) const;

    enum {
        fgStatusInfinite,
        fgStatusLength      // Leave last in list.
    } StatusFlags;

    UBool subparse(const UnicodeString& text, 
                   const UnicodeString* negPrefix,
                   const UnicodeString* negSuffix,
                   const UnicodeString* posPrefix,
                   const UnicodeString* posSuffix,
                   UBool currencyParsing,
                   int8_t type,
                   ParsePosition& parsePosition,
                   DigitList& digits, UBool* status,
                   UChar* currency) const;

    // Mixed style parsing for currency.
    // It parses against the current currency pattern 
    // using complex affix comparison
    // parses against the currency plural patterns using complex affix comparison,
    // and parses against the current pattern using simple affix comparison.
    UBool parseForCurrency(const UnicodeString& text, 
                           ParsePosition& parsePosition,
                           DigitList& digits,
                           UBool* status,
                           UChar* currency) const;

    int32_t skipPadding(const UnicodeString& text, int32_t position) const;

    int32_t compareAffix(const UnicodeString& input,
                         int32_t pos,
                         UBool isNegative,
                         UBool isPrefix,
                         const UnicodeString* affixPat,
                         UBool currencyParsing,
                         int8_t type,
                         UChar* currency) const;
    
    static int32_t compareSimpleAffix(const UnicodeString& affix,
                                      const UnicodeString& input,
                                      int32_t pos);
    
    static int32_t skipRuleWhiteSpace(const UnicodeString& text, int32_t pos);
    
    static int32_t skipUWhiteSpace(const UnicodeString& text, int32_t pos);
    
    int32_t compareComplexAffix(const UnicodeString& affixPat,
                                const UnicodeString& input,
                                int32_t pos,
                                int8_t type,
                                UChar* currency) const;

    static int32_t match(const UnicodeString& text, int32_t pos, UChar32 ch);

    static int32_t match(const UnicodeString& text, int32_t pos, const UnicodeString& str);

    /**
     * Get a decimal format symbol.
     * Returns a const reference to the symbol string.
     * @internal
     */
    inline const UnicodeString &getConstSymbol(DecimalFormatSymbols::ENumberFormatSymbol symbol) const;

    int32_t appendAffix(UnicodeString& buf, double number,
                        UBool isNegative, UBool isPrefix) const;

    /**
     * Append an affix to the given UnicodeString, using quotes if
     * there are special characters.  Single quotes themselves must be
     * escaped in either case.
     */
    void appendAffixPattern(UnicodeString& appendTo, const UnicodeString& affix,
                            UBool localized) const;

    void appendAffixPattern(UnicodeString& appendTo,
                            const UnicodeString* affixPattern,
                            const UnicodeString& expAffix, UBool localized) const;

    void expandAffix(const UnicodeString& pattern,
                     UnicodeString& affix,
                     double number,
                     UBool doFormat,
                     const UnicodeString* pluralCount) const;

    void expandAffixes(const UnicodeString* pluralCount);
    
    static double round(double a, ERoundingMode mode, UBool isNegative);

    void addPadding(UnicodeString& appendTo,
                    FieldPosition& fieldPosition,
                    int32_t prefixLen, int32_t suffixLen) const;

    UBool isGroupingPosition(int32_t pos) const;

    void setCurrencyForSymbols();

    // similar to setCurrency without re-compute the affixes for currency.
    // If currency changes, the affix pattern for currency is not changed,
    // but the affix will be changed. So, affixes need to be 
    // re-computed in setCurrency(), but not in setCurrencyInternally().
    virtual void setCurrencyInternally(const UChar* theCurrency, UErrorCode& ec);

    // set up currency affix patterns for mix parsing.
    // The patterns saved here are the affix patterns of default currency
    // pattern and the unique affix patterns of the plural currency patterns.
    // Those patterns are used by parseForCurrency().
    void setupCurrencyAffixPatterns(UErrorCode& status);

    // set up the currency affixes used in currency plural formatting.
    // It sets up both fAffixesForCurrency for currency pattern if the current
    // pattern contains 3 currency signs, 
    // and it sets up fPluralAffixesForCurrency for currency plural patterns.
    void setupCurrencyAffixes(const UnicodeString& pattern, 
                              UBool setupForCurrentPattern,
                              UBool setupForPluralPattern,
                              UErrorCode& status);
   
    // hashtable operations
    Hashtable* initHashForAffixPattern(UErrorCode& status);
    Hashtable* initHashForAffix(UErrorCode& status);

    void deleteHashForAffixPattern();
    void deleteHashForAffix(Hashtable*& table);

    void copyHashForAffixPattern(const Hashtable* source,
                                 Hashtable* target, UErrorCode& status);
    void copyHashForAffix(const Hashtable* source,
                          Hashtable* target, UErrorCode& status);

    // currency sign count
    enum {
        fgCurrencySignCountZero,
        fgCurrencySignCountInSymbolFormat,
        fgCurrencySignCountInISOFormat,
        fgCurrencySignCountInPluralFormat
    } CurrencySignCount;

    /**
     * Constants.
     */
    //static const int8_t fgMaxDigit; // The largest digit, in this case 9

    /*transient*/ //DigitList* fDigitList;

    UnicodeString           fPositivePrefix;
    UnicodeString           fPositiveSuffix;
    UnicodeString           fNegativePrefix;
    UnicodeString           fNegativeSuffix;
    UnicodeString*          fPosPrefixPattern;
    UnicodeString*          fPosSuffixPattern;
    UnicodeString*          fNegPrefixPattern;
    UnicodeString*          fNegSuffixPattern;

    /**
     * Formatter for ChoiceFormat-based currency names.  If this field
     * is not null, then delegate to it to format currency symbols.
     * @since ICU 2.6
     */
    ChoiceFormat*           fCurrencyChoice;

    int32_t                 fMultiplier;
    int32_t                 fGroupingSize;
    int32_t                 fGroupingSize2;
    UBool                   fDecimalSeparatorAlwaysShown;
    DecimalFormatSymbols*   fSymbols;

    UBool                   fUseSignificantDigits;
    int32_t                 fMinSignificantDigits;
    int32_t                 fMaxSignificantDigits;

    UBool                   fUseExponentialNotation;
    int8_t                  fMinExponentDigits;
    UBool                   fExponentSignAlwaysShown;

    /* If fRoundingIncrement is NULL, there is no rounding.  Otherwise, round to
     * fRoundingIncrement.getDouble().  Since this operation may be expensive,
     * we cache the result in fRoundingDouble.  All methods that update
     * fRoundingIncrement also update fRoundingDouble. */
    DigitList*              fRoundingIncrement;
    /*transient*/ double    fRoundingDouble;
    ERoundingMode           fRoundingMode;

    UChar32                 fPad;
    int32_t                 fFormatWidth;
    EPadPosition            fPadPosition;

    /*
     * Following are used for currency format
     */
    // pattern used in this formatter
    UnicodeString fFormatPattern;
    // style is only valid when decimal formatter is constructed by
    // DecimalFormat(pattern, decimalFormatSymbol, style)
    int fStyle;
    /*
     * Represents whether this is a currency format, and which
     * currency format style.
     * 0: not currency format type;
     * 1: currency style -- symbol name, such as "$" for US dollar.
     * 2: currency style -- ISO name, such as USD for US dollar.
     * 3: currency style -- plural long name, such as "US Dollar" for
     *                      "1.00 US Dollar", or "US Dollars" for
     *                      "3.00 US Dollars".
     */
    int fCurrencySignCount;


    /* For currency parsing purose,
     * Need to remember all prefix patterns and suffix patterns of 
     * every currency format pattern, 
     * including the pattern of default currecny style
     * and plural currency style. And the patterns are set through applyPattern.
     */
    // TODO: innerclass?
    struct AffixPatternsForCurrency : public UMemory {
        // negative prefix pattern
        UnicodeString negPrefixPatternForCurrency;
        // negative suffix pattern
        UnicodeString negSuffixPatternForCurrency;
        // positive prefix pattern
        UnicodeString posPrefixPatternForCurrency;
        // positive suffix pattern
        UnicodeString posSuffixPatternForCurrency;
        int8_t patternType;
        
        AffixPatternsForCurrency(const UnicodeString& negPrefix, 
                                 const UnicodeString& negSuffix,
                                 const UnicodeString& posPrefix,
                                 const UnicodeString& posSuffix,
                                 int8_t type) {
            negPrefixPatternForCurrency = negPrefix;
            negSuffixPatternForCurrency = negSuffix;
            posPrefixPatternForCurrency = posPrefix;
            posSuffixPatternForCurrency = posSuffix;
            patternType = type;
        }
    };
        
    /* affix for currency formatting when the currency sign in the pattern
     * equals to 3, such as the pattern contains 3 currency sign or 
     * the formatter style is currency plural format style.
     */
    struct AffixesForCurrency : public UMemory {
        // negative prefix
        UnicodeString negPrefixForCurrency;
        // negative suffix
        UnicodeString negSuffixForCurrency;
        // positive prefix
        UnicodeString posPrefixForCurrency;
        // positive suffix
        UnicodeString posSuffixForCurrency;
        
        int32_t formatWidth;

        AffixesForCurrency(const UnicodeString& negPrefix,
                           const UnicodeString& negSuffix,
                           const UnicodeString& posPrefix,
                           const UnicodeString& posSuffix) {
            negPrefixForCurrency = negPrefix;
            negSuffixForCurrency = negSuffix;
            posPrefixForCurrency = posPrefix;
            posSuffixForCurrency = posSuffix;
        }
    };

    // Affix pattern set for currency.
    // It is a set of AffixPatternsForCurrency,
    // each element of the set saves the negative prefix pattern,
    // negative suffix pattern, positive prefix pattern, 
    // and positive suffix  pattern of a pattern.
    // It is used for currency mixed style parsing.
    // It is actually is a set.
    // The set contains the default currency pattern from the locale,
    // and the currency plural patterns.
    // Since it is a set, it does not contain duplicated items.
    // For example, if 2 currency plural patterns are the same, only one pattern
    // is included in the set. When parsing, we do not check whether the plural
    // count match or not.
    Hashtable* fAffixPatternsForCurrency;

    // Following 2 are affixes for currency.
    // It is a hash map from plural count to AffixesForCurrency.
    // AffixesForCurrency saves the negative prefix,
    // negative suffix, positive prefix, and positive suffix of a pattern.
    // It is used during currency formatting only when the currency sign count
    // is 3. In which case, the affixes are getting from here, not
    // from the fNegativePrefix etc.
    Hashtable* fAffixesForCurrency;  // for current pattern
    Hashtable* fPluralAffixesForCurrency;  // for plural pattern

    // Information needed for DecimalFormat to format/parse currency plural.
    CurrencyPluralInfo* fCurrencyPluralInfo;

protected:

    /**
     * Returns the currency in effect for this formatter.  Subclasses
     * should override this method as needed.  Unlike getCurrency(),
     * this method should never return "".
     * @result output parameter for null-terminated result, which must
     * have a capacity of at least 4
     * @internal
     */
    virtual void getEffectiveCurrency(UChar* result, UErrorCode& ec) const;

  /** number of integer digits 
   * @stable ICU 2.4
   */  
    static const int32_t  kDoubleIntegerDigits;
  /** number of fraction digits 
   * @stable ICU 2.4
   */  
    static const int32_t  kDoubleFractionDigits;

    /**
     * When someone turns on scientific mode, we assume that more than this
     * number of digits is due to flipping from some other mode that didn't
     * restrict the maximum, and so we force 1 integer digit.  We don't bother
     * to track and see if someone is using exponential notation with more than
     * this number, it wouldn't make sense anyway, and this is just to make sure
     * that someone turning on scientific mode with default settings doesn't
     * end up with lots of zeroes.
     * @stable ICU 2.8
     */
    static const int32_t  kMaxScientificIntegerDigits;
};

inline UnicodeString&
DecimalFormat::format(const Formattable& obj,
                      UnicodeString& appendTo,
                      UErrorCode& status) const {
    // Don't use Format:: - use immediate base class only,
    // in case immediate base modifies behavior later.
    return NumberFormat::format(obj, appendTo, status);
}

inline UnicodeString&
DecimalFormat::format(double number,
                      UnicodeString& appendTo) const {
    FieldPosition pos(0);
    return format(number, appendTo, pos);
}

inline UnicodeString&
DecimalFormat::format(int32_t number,
                      UnicodeString& appendTo) const {
    FieldPosition pos(0);
    return format((int64_t)number, appendTo, pos);
}

inline const UnicodeString &
DecimalFormat::getConstSymbol(DecimalFormatSymbols::ENumberFormatSymbol symbol) const {
    return fSymbols->getConstSymbol(symbol);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _DECIMFMT
//eof
