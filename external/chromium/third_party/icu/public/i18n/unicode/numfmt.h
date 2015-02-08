
#ifndef NUMFMT_H
#define NUMFMT_H


#include "unicode/utypes.h"

 
#if !UCONFIG_NO_FORMATTING

#include "unicode/unistr.h"
#include "unicode/format.h"
#include "unicode/unum.h" // UNumberFormatStyle
#include "unicode/locid.h"

U_NAMESPACE_BEGIN

#if !UCONFIG_NO_SERVICE
class NumberFormatFactory;
class StringEnumeration;
#endif

class U_I18N_API NumberFormat : public Format {
public:

    /**
     * Constants for various number format styles.
     * kNumberStyle specifies a normal number style of format.
     * kCurrencyStyle specifies a currency format using currency symbol name,
     * such as in "$1.00".
     * kPercentStyle specifies a style of format to display percent.
     * kScientificStyle specifies a style of format to display scientific number.
     * kISOCurrencyStyle specifies a currency format using ISO currency code,
     * such as in "USD1.00".
     * kPluralCurrencyStyle specifies a currency format using currency plural
     * names, such as in "1.00 US dollar" and "3.00 US dollars".
     * @draft ICU 4.2
     */
    enum EStyles {
        kNumberStyle,
        kCurrencyStyle,
        kPercentStyle,
        kScientificStyle,
        kIsoCurrencyStyle,
        kPluralCurrencyStyle,
        kStyleCount // ALWAYS LAST ENUM: number of styles
    };

    /**
     * Alignment Field constants used to construct a FieldPosition object.
     * Signifies that the position of the integer part or fraction part of
     * a formatted number should be returned.
     *
     * @see FieldPosition
     * @stable ICU 2.0
     */
    enum EAlignmentFields {
        kIntegerField,
        kFractionField,


    /**
     * These constants are provided for backwards compatibility only.
     * Please use the C++ style constants defined above.
     * @stable ICU 2.0
     */
        INTEGER_FIELD        = kIntegerField,
        FRACTION_FIELD        = kFractionField
    };

    /**
     * Destructor.
     * @stable ICU 2.0
     */
    virtual ~NumberFormat();

    /**
     * Return true if the given Format objects are semantically equal.
     * Objects of different subclasses are considered unequal.
     * @return    true if the given Format objects are semantically equal.
     * @stable ICU 2.0
     */
    virtual UBool operator==(const Format& other) const;

    /**
     * Format an object to produce a string.  This method handles
     * Formattable objects with numeric types. If the Formattable
     * object type is not a numeric type, then it returns a failing
     * UErrorCode.
     *
     * @param obj       The object to format.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param pos       On input: an alignment field, if desired.
     *                  On output: the offsets of the alignment field.
     * @param status    Output param filled with success/failure status.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    virtual UnicodeString& format(const Formattable& obj,
                                  UnicodeString& appendTo,
                                  FieldPosition& pos,
                                  UErrorCode& status) const;

    /**
     * Parse a string to produce an object.  This methods handles
     * parsing of numeric strings into Formattable objects with numeric
     * types.
     * <P>
     * Before calling, set parse_pos.index to the offset you want to
     * start parsing at in the source. After calling, parse_pos.index
     * indicates the position after the successfully parsed text.  If
     * an error occurs, parse_pos.index is unchanged.
     * <P>
     * When parsing, leading whitespace is discarded (with successful
     * parse), while trailing whitespace is left as is.
     * <P>
     * See Format::parseObject() for more.
     *
     * @param source    The string to be parsed into an object.
     * @param result    Formattable to be set to the parse result.
     *                  If parse fails, return contents are undefined.
     * @param parse_pos The position to start parsing at. Upon return
     *                  this param is set to the position after the
     *                  last character successfully parsed. If the
     *                  source is not parsed successfully, this param
     *                  will remain unchanged.
     * @return          A newly created Formattable* object, or NULL
     *                  on failure.  The caller owns this and should
     *                  delete it when done.
     * @stable ICU 2.0
     */
    virtual void parseObject(const UnicodeString& source,
                             Formattable& result,
                             ParsePosition& parse_pos) const;

    /**
     * Format a double number. These methods call the NumberFormat
     * pure virtual format() methods with the default FieldPosition.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    UnicodeString& format(  double number,
                            UnicodeString& appendTo) const;

    /**
     * Format a long number. These methods call the NumberFormat
     * pure virtual format() methods with the default FieldPosition.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    UnicodeString& format(  int32_t number,
                            UnicodeString& appendTo) const;

    /**
     * Format an int64 number. These methods call the NumberFormat
     * pure virtual format() methods with the default FieldPosition.
     *
     * @param number    The value to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.8
     */
    UnicodeString& format(  int64_t number,
                            UnicodeString& appendTo) const;

    /**
     * Format a double number. Concrete subclasses must implement
     * these pure virtual methods.
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
                                  FieldPosition& pos) const = 0;
    /**
     * Format a long number. Concrete subclasses must implement
     * these pure virtual methods.
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
                                  FieldPosition& pos) const = 0;

    /**
     * Format an int64 number. (Not abstract to retain compatibility
     * with earlier releases, however subclasses should override this
     * method as it just delegates to format(int32_t number...);
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
     * Redeclared Format method.
     * @param obj       The object to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param status    Output parameter set to a failure error code
     *                  when a failure occurs.
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    UnicodeString& format(const Formattable& obj,
                          UnicodeString& appendTo,
                          UErrorCode& status) const;

   /**
    * Return a long if possible (e.g. within range LONG_MAX,
    * LONG_MAX], and with no decimals), otherwise a double.  If
    * IntegerOnly is set, will stop at a decimal point (or equivalent;
    * e.g. for rational numbers "1 2/3", will stop after the 1).
    * <P>
    * If no object can be parsed, index is unchanged, and NULL is
    * returned.
    * <P>
    * This is a pure virtual which concrete subclasses must implement.
    *
    * @param text           The text to be parsed.
    * @param result         Formattable to be set to the parse result.
    *                       If parse fails, return contents are undefined.
    * @param parsePosition  The position to start parsing at on input.
    *                       On output, moved to after the last successfully
    *                       parse character. On parse failure, does not change.
    * @return               A Formattable object of numeric type.  The caller
    *                       owns this an must delete it.  NULL on failure.
    * @stable ICU 2.0
    */
    virtual void parse(const UnicodeString& text,
                       Formattable& result,
                       ParsePosition& parsePosition) const = 0;

    /**
     * Parse a string as a numeric value, and return a Formattable
     * numeric object. This method parses integers only if IntegerOnly
     * is set.
     *
     * @param text          The text to be parsed.
     * @param result        Formattable to be set to the parse result.
     *                      If parse fails, return contents are undefined.
     * @param status        Output parameter set to a failure error code
     *                      when a failure occurs.
     * @return              A Formattable object of numeric type.  The caller
     *                      owns this an must delete it.  NULL on failure.
     * @see                 NumberFormat::isParseIntegerOnly
     * @stable ICU 2.0
     */
    virtual void parse( const UnicodeString& text,
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
     * Return true if this format will parse numbers as integers
     * only.  For example in the English locale, with ParseIntegerOnly
     * true, the string "1234." would be parsed as the integer value
     * 1234 and parsing would stop at the "." character.  Of course,
     * the exact format accepted by the parse operation is locale
     * dependant and determined by sub-classes of NumberFormat.
     * @return    true if this format will parse numbers as integers
     *            only.
     * @stable ICU 2.0
     */
    UBool isParseIntegerOnly(void) const;

    /**
     * Sets whether or not numbers should be parsed as integers only.
     * @param value    set True, this format will parse numbers as integers
     *                 only.
     * @see isParseIntegerOnly
     * @stable ICU 2.0
     */
    virtual void setParseIntegerOnly(UBool value);

    /**
     * Returns the default number format for the current default
     * locale.  The default format is one of the styles provided by
     * the other factory methods: getNumberInstance,
     * getCurrencyInstance or getPercentInstance.  Exactly which one
     * is locale dependant.
     * @stable ICU 2.0
     */
    static NumberFormat* U_EXPORT2 createInstance(UErrorCode&);

    /**
     * Returns the default number format for the specified locale.
     * The default format is one of the styles provided by the other
     * factory methods: getNumberInstance, getCurrencyInstance or
     * getPercentInstance.  Exactly which one is locale dependant.
     * @param inLocale    the given locale.
     * @stable ICU 2.0
     */
    static NumberFormat* U_EXPORT2 createInstance(const Locale& inLocale,
                                        UErrorCode&);

    /**
     * Creates the specified decimal format style of the desired locale.
     * @param desiredLocale    the given locale.
     * @param choice           the given style.
     * @param success          Output param filled with success/failure status.
     * @return                 A new NumberFormat instance.
     * @draft ICU 4.2
     */
    static NumberFormat* U_EXPORT2 createInstance(const Locale& desiredLocale, EStyles choice, UErrorCode& success);


    /**
     * Returns a currency format for the current default locale.
     * @stable ICU 2.0
     */
    static NumberFormat* U_EXPORT2 createCurrencyInstance(UErrorCode&);

    /**
     * Returns a currency format for the specified locale.
     * @param inLocale    the given locale.
     * @stable ICU 2.0
     */
    static NumberFormat* U_EXPORT2 createCurrencyInstance(const Locale& inLocale,
                                                UErrorCode&);

    /**
     * Returns a percentage format for the current default locale.
     * @stable ICU 2.0
     */
    static NumberFormat* U_EXPORT2 createPercentInstance(UErrorCode&);

    /**
     * Returns a percentage format for the specified locale.
     * @param inLocale    the given locale.
     * @stable ICU 2.0
     */
    static NumberFormat* U_EXPORT2 createPercentInstance(const Locale& inLocale,
                                               UErrorCode&);

    /**
     * Returns a scientific format for the current default locale.
     * @stable ICU 2.0
     */
    static NumberFormat* U_EXPORT2 createScientificInstance(UErrorCode&);

    /**
     * Returns a scientific format for the specified locale.
     * @param inLocale    the given locale.
     * @stable ICU 2.0
     */
    static NumberFormat* U_EXPORT2 createScientificInstance(const Locale& inLocale,
                                                UErrorCode&);

    /**
     * Get the set of Locales for which NumberFormats are installed.
     * @param count    Output param to receive the size of the locales
     * @stable ICU 2.0
     */
    static const Locale* U_EXPORT2 getAvailableLocales(int32_t& count);

#if !UCONFIG_NO_SERVICE
    /**
     * Register a new NumberFormatFactory.  The factory will be adopted.
     * @param toAdopt the NumberFormatFactory instance to be adopted
     * @param status the in/out status code, no special meanings are assigned
     * @return a registry key that can be used to unregister this factory
     * @stable ICU 2.6
     */
    static URegistryKey U_EXPORT2 registerFactory(NumberFormatFactory* toAdopt, UErrorCode& status);

    /**
     * Unregister a previously-registered NumberFormatFactory using the key returned from the
     * register call.  Key becomes invalid after a successful call and should not be used again.
     * The NumberFormatFactory corresponding to the key will be deleted.
     * @param key the registry key returned by a previous call to registerFactory
     * @param status the in/out status code, no special meanings are assigned
     * @return TRUE if the factory for the key was successfully unregistered
     * @stable ICU 2.6
     */
    static UBool U_EXPORT2 unregister(URegistryKey key, UErrorCode& status);

    /**
     * Return a StringEnumeration over the locales available at the time of the call,
     * including registered locales.
     * @return a StringEnumeration over the locales available at the time of the call
     * @stable ICU 2.6
     */
    static StringEnumeration* U_EXPORT2 getAvailableLocales(void);
#endif /* UCONFIG_NO_SERVICE */

    /**
     * Returns true if grouping is used in this format. For example,
     * in the English locale, with grouping on, the number 1234567
     * might be formatted as "1,234,567". The grouping separator as
     * well as the size of each group is locale dependant and is
     * determined by sub-classes of NumberFormat.
     * @see setGroupingUsed
     * @stable ICU 2.0
     */
    UBool isGroupingUsed(void) const;

    /**
     * Set whether or not grouping will be used in this format.
     * @param newValue    True, grouping will be used in this format.
     * @see getGroupingUsed
     * @stable ICU 2.0
     */
    virtual void setGroupingUsed(UBool newValue);

    /**
     * Returns the maximum number of digits allowed in the integer portion of a
     * number.
     * @return     the maximum number of digits allowed in the integer portion of a
     *             number.
     * @see setMaximumIntegerDigits
     * @stable ICU 2.0
     */
    int32_t getMaximumIntegerDigits(void) const;

    /**
     * Sets the maximum number of digits allowed in the integer portion of a
     * number. maximumIntegerDigits must be >= minimumIntegerDigits.  If the
     * new value for maximumIntegerDigits is less than the current value
     * of minimumIntegerDigits, then minimumIntegerDigits will also be set to
     * the new value.
     *
     * @param newValue    the new value for the maximum number of digits
     *                    allowed in the integer portion of a number.
     * @see getMaximumIntegerDigits
     * @stable ICU 2.0
     */
    virtual void setMaximumIntegerDigits(int32_t newValue);

    /**
     * Returns the minimum number of digits allowed in the integer portion of a
     * number.
     * @return    the minimum number of digits allowed in the integer portion of a
     *            number.
     * @see setMinimumIntegerDigits
     * @stable ICU 2.0
     */
    int32_t getMinimumIntegerDigits(void) const;

    /**
     * Sets the minimum number of digits allowed in the integer portion of a
     * number. minimumIntegerDigits must be &lt;= maximumIntegerDigits.  If the
     * new value for minimumIntegerDigits exceeds the current value
     * of maximumIntegerDigits, then maximumIntegerDigits will also be set to
     * the new value.
     * @param newValue    the new value to be set.
     * @see getMinimumIntegerDigits
     * @stable ICU 2.0
     */
    virtual void setMinimumIntegerDigits(int32_t newValue);

    /**
     * Returns the maximum number of digits allowed in the fraction portion of a
     * number.
     * @return    the maximum number of digits allowed in the fraction portion of a
     *            number.
     * @see setMaximumFractionDigits
     * @stable ICU 2.0
     */
    int32_t getMaximumFractionDigits(void) const;

    /**
     * Sets the maximum number of digits allowed in the fraction portion of a
     * number. maximumFractionDigits must be >= minimumFractionDigits.  If the
     * new value for maximumFractionDigits is less than the current value
     * of minimumFractionDigits, then minimumFractionDigits will also be set to
     * the new value.
     * @param newValue    the new value to be set.
     * @see getMaximumFractionDigits
     * @stable ICU 2.0
     */
    virtual void setMaximumFractionDigits(int32_t newValue);

    /**
     * Returns the minimum number of digits allowed in the fraction portion of a
     * number.
     * @return    the minimum number of digits allowed in the fraction portion of a
     *            number.
     * @see setMinimumFractionDigits
     * @stable ICU 2.0
     */
    int32_t getMinimumFractionDigits(void) const;

    /**
     * Sets the minimum number of digits allowed in the fraction portion of a
     * number. minimumFractionDigits must be &lt;= maximumFractionDigits.   If the
     * new value for minimumFractionDigits exceeds the current value
     * of maximumFractionDigits, then maximumIntegerDigits will also be set to
     * the new value
     * @param newValue    the new value to be set.
     * @see getMinimumFractionDigits
     * @stable ICU 2.0
     */
    virtual void setMinimumFractionDigits(int32_t newValue);

    /**
     * Sets the currency used to display currency
     * amounts.  This takes effect immediately, if this format is a
     * currency format.  If this format is not a currency format, then
     * the currency is used if and when this object becomes a
     * currency format.
     * @param theCurrency a 3-letter ISO code indicating new currency
     * to use.  It need not be null-terminated.  May be the empty
     * string or NULL to indicate no currency.
     * @param ec input-output error code
     * @stable ICU 3.0
     */
    virtual void setCurrency(const UChar* theCurrency, UErrorCode& ec);

    /**
     * Gets the currency used to display currency
     * amounts.  This may be an empty string for some subclasses.
     * @return a 3-letter null-terminated ISO code indicating
     * the currency in use, or a pointer to the empty string.
     * @stable ICU 2.6
     */
    const UChar* getCurrency() const;

public:

    /**
     * Return the class ID for this class.  This is useful for
     * comparing to a return value from getDynamicClassID(). Note that,
     * because NumberFormat is an abstract base class, no fully constructed object
     * will have the class ID returned by NumberFormat::getStaticClassID().
     * @return The class ID for all objects of this class.
     * @stable ICU 2.0
     */
    static UClassID U_EXPORT2 getStaticClassID(void);

    /**
     * Returns a unique class ID POLYMORPHICALLY.  Pure virtual override.
     * This method is to implement a simple version of RTTI, since not all
     * C++ compilers support genuine RTTI.  Polymorphic operator==() and
     * clone() methods call this method.
     * <P>
     * @return The class ID for this object. All objects of a
     * given class have the same class ID.  Objects of
     * other classes have different class IDs.
     * @stable ICU 2.0
     */
    virtual UClassID getDynamicClassID(void) const = 0;

protected:

    /**
     * Default constructor for subclass use only.
     * @stable ICU 2.0
     */
    NumberFormat();

    /**
     * Copy constructor.
     * @stable ICU 2.0
     */
    NumberFormat(const NumberFormat&);

    /**
     * Assignment operator.
     * @stable ICU 2.0
     */
    NumberFormat& operator=(const NumberFormat&);

    /**
     * Returns the currency in effect for this formatter.  Subclasses
     * should override this method as needed.  Unlike getCurrency(),
     * this method should never return "".
     * @result output parameter for null-terminated result, which must
     * have a capacity of at least 4
     * @internal
     */
    virtual void getEffectiveCurrency(UChar* result, UErrorCode& ec) const;

private:

    /**
     * Creates the specified decimal format style of the desired locale.
     * @param desiredLocale    the given locale.
     * @param choice           the given style.
     * @param success          Output param filled with success/failure status.
     * @return                 A new NumberFormat instance.
     */
    static NumberFormat* makeInstance(const Locale& desiredLocale, EStyles choice, UErrorCode& success);

    UBool      fGroupingUsed;
    int32_t     fMaxIntegerDigits;
    int32_t     fMinIntegerDigits;
    int32_t     fMaxFractionDigits;
    int32_t     fMinFractionDigits;
    UBool      fParseIntegerOnly;

    // ISO currency code
    UChar      fCurrency[4];

    friend class ICUNumberFormatFactory; // access to makeInstance, EStyles
    friend class ICUNumberFormatService;
};

#if !UCONFIG_NO_SERVICE
class U_I18N_API NumberFormatFactory : public UObject {
public:

    /**
     * Destructor
     * @stable ICU 3.0
     */
    virtual ~NumberFormatFactory();

    /**
     * Return true if this factory will be visible.  Default is true.
     * If not visible, the locales supported by this factory will not
     * be listed by getAvailableLocales.
     * @stable ICU 2.6
     */
    virtual UBool visible(void) const = 0;

    /**
     * Return the locale names directly supported by this factory.  The number of names
     * is returned in count;
     * @stable ICU 2.6
     */
    virtual const UnicodeString * getSupportedIDs(int32_t &count, UErrorCode& status) const = 0;

    /**
     * Return a number format of the appropriate type.  If the locale
     * is not supported, return null.  If the locale is supported, but
     * the type is not provided by this service, return null.  Otherwise
     * return an appropriate instance of NumberFormat.
     * @stable ICU 2.6
     */
    virtual NumberFormat* createFormat(const Locale& loc, UNumberFormatStyle formatType) = 0;
};

class U_I18N_API SimpleNumberFormatFactory : public NumberFormatFactory {
protected:
    /**
     * True if the locale supported by this factory is visible.
     * @stable ICU 2.6
     */
    const UBool _visible;

    /**
     * The locale supported by this factory, as a UnicodeString.
     * @stable ICU 2.6
     */
    UnicodeString _id;

public:
    /**
     * @stable ICU 2.6
     */
    SimpleNumberFormatFactory(const Locale& locale, UBool visible = TRUE);

    /**
     * @stable ICU 3.0
     */
    virtual ~SimpleNumberFormatFactory();

    /**
     * @stable ICU 2.6
     */
    virtual UBool visible(void) const;

    /**
     * @stable ICU 2.6
     */
    virtual const UnicodeString * getSupportedIDs(int32_t &count, UErrorCode& status) const;
};
#endif /* #if !UCONFIG_NO_SERVICE */

// -------------------------------------

inline UBool
NumberFormat::isParseIntegerOnly() const
{
    return fParseIntegerOnly;
}

inline UnicodeString&
NumberFormat::format(const Formattable& obj,
                     UnicodeString& appendTo,
                     UErrorCode& status) const {
    return Format::format(obj, appendTo, status);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _NUMFMT
//eof
