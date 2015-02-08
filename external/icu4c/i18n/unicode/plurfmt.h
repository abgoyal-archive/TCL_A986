
#ifndef PLURFMT
#define PLURFMT

#include "unicode/utypes.h"


#if !UCONFIG_NO_FORMATTING

#include "unicode/numfmt.h"
#include "unicode/plurrule.h"

U_NAMESPACE_BEGIN

class Hashtable;


class U_I18N_API PluralFormat : public Format {
public:

    /**
     * Creates a new <code>PluralFormat</code> for the default locale.
     * This locale will be used to get the set of plural rules and for standard
     * number formatting.
     * @param status  output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    PluralFormat(UErrorCode& status);

    /**
     * Creates a new <code>PluralFormat</code> for a given locale.
     * @param locale the <code>PluralFormat</code> will be configured with
     *               rules for this locale. This locale will also be used for
     *               standard number formatting.
     * @param status output param set to success/failure code on exit, which
     *               must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    PluralFormat(const Locale& locale, UErrorCode& status);

    /**
     * Creates a new <code>PluralFormat</code> for a given set of rules.
     * The standard number formatting will be done using the default locale.
     * @param rules   defines the behavior of the <code>PluralFormat</code>
     *                object.
     * @param status  output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    PluralFormat(const PluralRules& rules, UErrorCode& status);

    /**
     * Creates a new <code>PluralFormat</code> for a given set of rules.
     * The standard number formatting will be done using the given locale.
     * @param locale  the default number formatting will be done using this
     *                locale.
     * @param rules   defines the behavior of the <code>PluralFormat</code>
     *                object.
     * @param status  output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    PluralFormat(const Locale& locale, const PluralRules& rules, UErrorCode& status);

    /**
     * Creates a new <code>PluralFormat</code> for a given pattern string.
     * The default locale will be used to get the set of plural rules and for
     * standard number formatting.
     * @param  pattern the pattern for this <code>PluralFormat</code>.
     *                 errors are returned to status if the pattern is invalid.
     * @param status   output param set to success/failure code on exit, which
     *                 must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    PluralFormat(const UnicodeString& pattern, UErrorCode& status);

    /**
     * Creates a new <code>PluralFormat</code> for a given pattern string and
     * locale.
     * The locale will be used to get the set of plural rules and for
     * standard number formatting.
     * @param locale   the <code>PluralFormat</code> will be configured with
     *                 rules for this locale. This locale will also be used for
     *                 standard number formatting.
     * @param pattern  the pattern for this <code>PluralFormat</code>.
     *                 errors are returned to status if the pattern is invalid.
     * @param status   output param set to success/failure code on exit, which
     *                 must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    PluralFormat(const Locale& locale, const UnicodeString& pattern, UErrorCode& status);

    /**
     * Creates a new <code>PluralFormat</code> for a given set of rules, a
     * pattern and a locale.
     * @param rules    defines the behavior of the <code>PluralFormat</code>
     *                 object.
     * @param pattern  the pattern for this <code>PluralFormat</code>.
     *                 errors are returned to status if the pattern is invalid.
     * @param status   output param set to success/failure code on exit, which
     *                 must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    PluralFormat(const PluralRules& rules,
                 const UnicodeString& pattern,
                 UErrorCode& status);

    /**
     * Creates a new <code>PluralFormat</code> for a given set of rules, a
     * pattern and a locale.
     * @param locale  the <code>PluralFormat</code> will be configured with
     *                rules for this locale. This locale will also be used for
     *                standard number formatting.
     * @param rules   defines the behavior of the <code>PluralFormat</code>
     *                object.
     * @param pattern the pattern for this <code>PluralFormat</code>.
     *                errors are returned to status if the pattern is invalid.
     * @param status  output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    PluralFormat(const Locale& locale,
                 const PluralRules& rules,
                 const UnicodeString& pattern,
                 UErrorCode& status);

    /**
      * copy constructor.
      * @stable ICU 4.0
      */
    PluralFormat(const PluralFormat& other);

    /**
     * Destructor.
     * @stable ICU 4.0
     */
    virtual ~PluralFormat();

    /**
     * Sets the pattern used by this plural format.
     * The method parses the pattern and creates a map of format strings
     * for the plural rules.
     * Patterns and their interpretation are specified in the class description.
     *
     * @param pattern the pattern for this plural format
     *                errors are returned to status if the pattern is invalid.
     * @param status  output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    void applyPattern(const UnicodeString& pattern, UErrorCode& status);


    using Format::format;

    /**
     * Formats a plural message for a given number.
     *
     * @param number  a number for which the plural message should be formatted
     *                for. If no pattern has been applied to this
     *                <code>PluralFormat</code> object yet, the formatted number
     *                will be returned.
     * @param status  output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @return        the string containing the formatted plural message.
     * @stable ICU 4.0
     */
    UnicodeString format(int32_t number, UErrorCode& status) const;

    /**
     * Formats a plural message for a given number.
     *
     * @param number  a number for which the plural message should be formatted
     *                for. If no pattern has been applied to this
     *                PluralFormat object yet, the formatted number
     *                will be returned.
     * @param status  output param set to success or failure code on exit, which
     *                must not indicate a failure before the function call.
     * @return        the string containing the formatted plural message.
     * @stable ICU 4.0
     */
    UnicodeString format(double number, UErrorCode& status) const;

    /**
     * Formats a plural message for a given number.
     *
     * @param number   a number for which the plural message should be formatted
     *                 for. If no pattern has been applied to this
     *                 <code>PluralFormat</code> object yet, the formatted number
     *                 will be returned.
     * @param appendTo output parameter to receive result.
     *                 result is appended to existing contents.
     * @param pos      On input: an alignment field, if desired.
     *                 On output: the offsets of the alignment field.
     * @param status   output param set to success/failure code on exit, which
     *                 must not indicate a failure before the function call.
     * @return         the string containing the formatted plural message.
     * @stable ICU 4.0
     */
    UnicodeString& format(int32_t number,
                          UnicodeString& appendTo,
                          FieldPosition& pos,
                          UErrorCode& status) const;

    /**
     * Formats a plural message for a given number.
     *
     * @param number   a number for which the plural message should be formatted
     *                 for. If no pattern has been applied to this
     *                 PluralFormat object yet, the formatted number
     *                 will be returned.
     * @param appendTo output parameter to receive result.
     *                 result is appended to existing contents.
     * @param pos      On input: an alignment field, if desired.
     *                 On output: the offsets of the alignment field.
     * @param status   output param set to success/failure code on exit, which
     *                 must not indicate a failure before the function call.
     * @return         the string containing the formatted plural message.
     * @stable ICU 4.0
     */
    UnicodeString& format(double number,
                          UnicodeString& appendTo,
                          FieldPosition& pos,
                          UErrorCode& status) const;

    /**
     * Sets the locale used by this <code>PluraFormat</code> object.
     * Note: Calling this method resets this <code>PluraFormat</code> object,
     *     i.e., a pattern that was applied previously will be removed,
     *     and the NumberFormat is set to the default number format for
     *     the locale.  The resulting format behaves the same as one
     *     constructed from {@link #PluralFormat(const Locale& locale, UErrorCode& status)}.
     * @param locale  the <code>locale</code> to use to configure the formatter.
     * @param status  output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @stable ICU 4.0
     */
    void setLocale(const Locale& locale, UErrorCode& status);

    /**
      * Sets the number format used by this formatter.  You only need to
      * call this if you want a different number format than the default
      * formatter for the locale.
      * @param format  the number format to use.
      * @param status  output param set to success/failure code on exit, which
      *                must not indicate a failure before the function call.
      * @stable ICU 4.0
      */
    void setNumberFormat(const NumberFormat* format, UErrorCode& status);

    /**
       * Assignment operator
       *
       * @param other    the PluralFormat object to copy from.
       * @stable ICU 4.0
       */
    PluralFormat& operator=(const PluralFormat& other);

    /**
      * Return true if another object is semantically equal to this one.
      *
      * @param other    the PluralFormat object to be compared with.
      * @return         true if other is semantically equal to this.
      * @stable ICU 4.0
      */
    virtual UBool operator==(const Format& other) const;

    /**
     * Return true if another object is semantically unequal to this one.
     *
     * @param other    the PluralFormat object to be compared with.
     * @return         true if other is semantically unequal to this.
     * @stable ICU 4.0
     */
    virtual UBool operator!=(const Format& other) const;

    /**
     * Clones this Format object polymorphically.  The caller owns the
     * result and should delete it when done.
     * @stable ICU 4.0
     */
    virtual Format* clone(void) const;

    /**
    * Redeclared Format method.
    *
    * @param obj       The object to be formatted into a string.
    * @param appendTo  output parameter to receive result.
    *                  Result is appended to existing contents.
    * @param pos       On input: an alignment field, if desired.
    *                  On output: the offsets of the alignment field.
    * @param status    output param filled with success/failure status.
    * @return          Reference to 'appendTo' parameter.
    * @stable ICU 4.0
    */
   UnicodeString& format(const Formattable& obj,
                         UnicodeString& appendTo,
                         FieldPosition& pos,
                         UErrorCode& status) const;

   /**
    * Returns the pattern from applyPattern() or constructor().
    *
    * @param  appendTo  output parameter to receive result.
     *                  Result is appended to existing contents.
    * @return the UnicodeString with inserted pattern.
    * @stable ICU 4.0
    */
   UnicodeString& toPattern(UnicodeString& appendTo);

   /**
    * This method is not yet supported by <code>PluralFormat</code>.
    * <P>
    * Before calling, set parse_pos.index to the offset you want to start
    * parsing at in the source. After calling, parse_pos.index is the end of
    * the text you parsed. If error occurs, index is unchanged.
    * <P>
    * When parsing, leading whitespace is discarded (with a successful parse),
    * while trailing whitespace is left as is.
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
    * @stable ICU 4.0
    */
   virtual void parseObject(const UnicodeString& source,
                            Formattable& result,
                            ParsePosition& parse_pos) const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     *
     * @stable ICU 4.0
     *
     */
    static UClassID U_EXPORT2 getStaticClassID(void);

    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     *
     * @stable ICU 4.0
     */
     virtual UClassID getDynamicClassID() const;

private:
    typedef enum fmtToken {
        none,
        tLetter,
        tNumber,
        tSpace,
        tNumberSign,
        tLeftBrace,
        tRightBrace
    }fmtToken;

    Locale  locale;
    PluralRules* pluralRules;
    UnicodeString pattern;
    Hashtable  *fParsedValuesHash;
    NumberFormat*  numberFormat;
    NumberFormat*  replacedNumberFormat;

    PluralFormat();   // default constructor not implemented
    void init(const PluralRules* rules, const Locale& curlocale, UErrorCode& status);
    UBool inRange(UChar ch, fmtToken& type);
    UBool checkSufficientDefinition();
    void parsingFailure();
    UnicodeString insertFormattedNumber(double number,
                                        UnicodeString& message,
                                        UnicodeString& appendTo,
                                        FieldPosition& pos) const;
    void copyHashtable(Hashtable *other, UErrorCode& status);
};

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _PLURFMT
//eof
