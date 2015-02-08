 
#ifndef CHOICFMT_H
#define CHOICFMT_H
 
#include "unicode/utypes.h"

 
#if !UCONFIG_NO_FORMATTING

#include "unicode/unistr.h"
#include "unicode/numfmt.h"
#include "unicode/fieldpos.h"
#include "unicode/format.h"

U_NAMESPACE_BEGIN

class MessageFormat;

class U_I18N_API ChoiceFormat: public NumberFormat {
public:
    /**
     * Construct a new ChoiceFormat with the limits and the corresponding formats
     * based on the pattern.
     *
     * @param pattern   Pattern used to construct object.
     * @param status    Output param to receive success code.  If the
     *                  pattern cannot be parsed, set to failure code.
     * @stable ICU 2.0
     */
    ChoiceFormat(const UnicodeString& pattern,
                 UErrorCode& status);


    /**
     * Construct a new ChoiceFormat with the given limits and formats.  Copy
     * the limits and formats instead of adopting them.
     *
     * @param limits    Array of limit values.
     * @param formats   Array of formats.
     * @param count     Size of 'limits' and 'formats' arrays.
     * @stable ICU 2.0
     */
    
    ChoiceFormat(const double* limits,
                 const UnicodeString* formats,
                 int32_t count );

    /**
     * Construct a new ChoiceFormat with the given limits and formats.
     * Copy the limits and formats (instead of adopting them).  By
     * default, each limit in the array specifies the inclusive lower
     * bound of its range, and the exclusive upper bound of the previous
     * range.  However, if the isLimitOpen element corresponding to a
     * limit is TRUE, then the limit is the exclusive lower bound of its
     * range, and the inclusive upper bound of the previous range.
     * @param limits Array of limit values
     * @param closures Array of booleans specifying whether each
     * element of 'limits' is open or closed.  If FALSE, then the
     * corresponding limit is a member of the range above it.  If TRUE,
     * then the limit belongs to the range below it.
     * @param formats Array of formats
     * @param count Size of 'limits', 'closures', and 'formats' arrays
     * @stable ICU 2.4
     */
    ChoiceFormat(const double* limits,
                 const UBool* closures,
                 const UnicodeString* formats,
                 int32_t count);

    /**
     * Copy constructor.
     *
     * @param that   ChoiceFormat object to be copied from
     * @stable ICU 2.0
     */
    ChoiceFormat(const ChoiceFormat& that);

    /**
     * Assignment operator.
     *
     * @param that   ChoiceFormat object to be copied
     * @stable ICU 2.0
     */
    const ChoiceFormat& operator=(const ChoiceFormat& that);

    /**
     * Destructor.
     * @stable ICU 2.0
     */
    virtual ~ChoiceFormat();

    /**
     * Clone this Format object polymorphically. The caller owns the
     * result and should delete it when done.
     *
     * @return a copy of this object
     * @stable ICU 2.0
     */
    virtual Format* clone(void) const;

    /**
     * Return true if the given Format objects are semantically equal.
     * Objects of different subclasses are considered unequal.
     *
     * @param other    ChoiceFormat object to be compared 
     * @return         true if other is the same as this. 
     * @stable ICU 2.0
     */
    virtual UBool operator==(const Format& other) const;

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
     * Sets the pattern.
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
     * Gets the pattern.
     * 
     * @param pattern    Output param which will recieve the pattern
     *                   Previous contents are deleted.
     * @return    A reference to 'pattern'
     * @stable ICU 2.0
     */
    virtual UnicodeString& toPattern(UnicodeString &pattern) const;

    /**
     * Set the choices to be used in formatting.
     *
     * @param limitsToCopy      Contains the top value that you want
     *                          parsed with that format,and should be in
     *                          ascending sorted order. When formatting X,
     *                          the choice will be the i, where limit[i]
     *                          &lt;= X &lt; limit[i+1].
     * @param formatsToCopy     The format strings you want to use for each limit.
     * @param count             The size of the above arrays.
     * @stable ICU 2.0
     */
    virtual void setChoices(const double* limitsToCopy,
                            const UnicodeString* formatsToCopy,
                            int32_t count );    

    /**
     * Set the choices to be used in formatting.  See class description
     * for documenatation of the limits, closures, and formats arrays.
     * @param limits Array of limits
     * @param closures Array of limit booleans
     * @param formats Array of format string
     * @param count The size of the above arrays
     * @stable ICU 2.4
     */
    virtual void setChoices(const double* limits,
                            const UBool* closures,
                            const UnicodeString* formats,
                            int32_t count);

    /**
     * Get the limits passed in the constructor.
     *
     * @param count    The size of the limits arrays
     * @return the limits.
     * @stable ICU 2.0
     */
    virtual const double* getLimits(int32_t& count) const;
    
    /**
     * Get the limit booleans passed in the constructor.  The caller
     * must not delete the result.
     *
     * @param count   The size of the arrays
     * @return the closures
     * @stable ICU 2.4
     */
    virtual const UBool* getClosures(int32_t& count) const;

    /**
     * Get the formats passed in the constructor.
     *
     * @param count   The size of the arrays
     * @return the formats.
     * @stable ICU 2.0
     */
    virtual const UnicodeString* getFormats(int32_t& count) const;

    /**
     * Format a double or long number using this object's choices.
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
     * Format a int_32t number using this object's choices.
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
     * Format an int64_t number using this object's choices.
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
     * Format an array of objects using this object's choices.
     *
     * @param objs      The array of objects to be formatted.
     * @param cnt       The size of objs.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param pos       On input: an alignment field, if desired.
     *                  On output: the offsets of the alignment field.
     * @param success   Output param set to success/failure code on
     *                  exit. 
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    virtual UnicodeString& format(const Formattable* objs,
                                  int32_t cnt,
                                  UnicodeString& appendTo,
                                  FieldPosition& pos,
                                  UErrorCode& success) const;
    /**
     * Format an object using this object's choices.
     *
     *
     * @param obj       The object to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param pos       On input: an alignment field, if desired.
     *                  On output: the offsets of the alignment field.
     * @param status    Output param set to success/failure code on
     *                  exit. 
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    virtual UnicodeString& format(const Formattable& obj,
                                  UnicodeString& appendTo,
                                  FieldPosition& pos, 
                                  UErrorCode& status) const;

    /**
     * Redeclared NumberFormat method.
     *
     * @param obj       The object to be formatted.
     * @param appendTo  Output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param status    Output param set to success/failure code on
     *                  exit. 
     * @return          Reference to 'appendTo' parameter.
     * @stable ICU 2.0
     */
    UnicodeString& format(const Formattable& obj,
                          UnicodeString& appendTo,
                          UErrorCode& status) const;

    /**
     * Redeclared NumberFormat method.
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
    UnicodeString& format(  int32_t number,
                            UnicodeString& appendTo) const;

   /**
    * Return a long if possible (e.g. within range LONG_MAX,
    * LONG_MAX], and with no decimals), otherwise a double.  If
    * IntegerOnly is set, will stop at a decimal point (or equivalent;
    * e.g. for rational numbers "1 2/3", will stop after the 1).
    * <P>
    * If no object can be parsed, parsePosition is unchanged, and NULL is
    * returned.
    *
    * @param text           The text to be parsed.
    * @param result         Formattable to be set to the parse result.
    *                       If parse fails, return contents are undefined.
    * @param parsePosition  The position to start parsing at on input.
    *                       On output, moved to after the last successfully
    *                       parse character. On parse failure, does not change.
    * @see                  NumberFormat::isParseIntegerOnly
    * @stable ICU 2.0
    */
    virtual void parse(const UnicodeString& text,
                       Formattable& result,
                       ParsePosition& parsePosition) const;
    
    /**
    * Return a long if possible (e.g. within range LONG_MAX,
    * LONG_MAX], and with no decimals), otherwise a double.  If
    * IntegerOnly is set, will stop at a decimal point (or equivalent;
    * e.g. for rational numbers "1 2/3", will stop after the 1).
    * <P>
    * If no object can be parsed, parsePosition is unchanged, and NULL is
    * returned.
    *
    * @param text           The text to be parsed.
    * @param result         Formattable to be set to the parse result.
    *                       If parse fails, return contents are undefined.
    * @param status         Output param with the formatted string.
    * @see                  NumberFormat::isParseIntegerOnly
    * @stable ICU 2.0
    */
    virtual void parse(const UnicodeString& text,
                       Formattable& result,
                       UErrorCode& status) const;
    
    
public:
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

    /**
     * Return the class ID for this class.  This is useful only for
     * comparing to a return value from getDynamicClassID().  For example:
     * <pre>
     * .       Base* polymorphic_pointer = createPolymorphicObject();
     * .       if (polymorphic_pointer->getDynamicClassID() ==
     * .           Derived::getStaticClassID()) ...
     * </pre>
     * @return          The class ID for all objects of this class.
     * @stable ICU 2.0
     */
    static UClassID U_EXPORT2 getStaticClassID(void);

private:
    // static cache management (thread-safe)
  //  static NumberFormat* getNumberFormat(UErrorCode &status); // call this function to 'check out' a numberformat from the cache.
  //  static void          releaseNumberFormat(NumberFormat *adopt); // call this function to 'return' the number format to the cache.
    
    /**
     * Converts a string to a double value using a default NumberFormat object
     * which is static (shared by all ChoiceFormat instances).
     * @param string the string to be converted with.
     * @return the converted double number.
     */
    static double stod(const UnicodeString& string);

    /**
     * Converts a double value to a string using a default NumberFormat object
     * which is static (shared by all ChoiceFormat instances).
     * @param value the double number to be converted with.
     * @param string the result string.
     * @return the converted string.
     */
    static UnicodeString& dtos(double value, UnicodeString& string);

    ChoiceFormat(); // default constructor not implemented

    /**
     * Construct a new ChoiceFormat with the limits and the corresponding formats
     * based on the pattern.
     *
     * @param newPattern   Pattern used to construct object.
     * @param parseError   Struct to recieve information on position 
     *                     of error if an error is encountered.
     * @param status       Output param to receive success code.  If the
     *                     pattern cannot be parsed, set to failure code.
     * @stable ICU 2.0
     */
    ChoiceFormat(const UnicodeString& newPattern,
                 UParseError& parseError,
                 UErrorCode& status);

    friend class MessageFormat;
    /**
     * Each ChoiceFormat divides the range -Inf..+Inf into fCount
     * intervals.  The intervals are:
     *
     *         0: fChoiceLimits[0]..fChoiceLimits[1]
     *         1: fChoiceLimits[1]..fChoiceLimits[2]
     *        ...
     *  fCount-2: fChoiceLimits[fCount-2]..fChoiceLimits[fCount-1]
     *  fCount-1: fChoiceLimits[fCount-1]..+Inf
     *
     * Interval 0 is special; during formatting (mapping numbers to
     * strings), it also contains all numbers less than
     * fChoiceLimits[0], as well as NaN values.
     *
     * Interval i maps to and from string fChoiceFormats[i].  When
     * parsing (mapping strings to numbers), then intervals map to
     * their lower limit, that is, interval i maps to fChoiceLimit[i].
     *
     * The intervals may be closed, half open, or open.  This affects
     * formatting but does not affect parsing.  Interval i is affected
     * by fClosures[i] and fClosures[i+1].  If fClosures[i]
     * is FALSE, then the value fChoiceLimits[i] is in interval i.
     * That is, intervals i and i are:
     *
     *  i-1:                 ... x < fChoiceLimits[i]
     *    i: fChoiceLimits[i] <= x ...
     *
     * If fClosures[i] is TRUE, then the value fChoiceLimits[i] is
     * in interval i-1.  That is, intervals i-1 and i are:
     *
     *  i-1:                ... x <= fChoiceLimits[i]
     *    i: fChoiceLimits[i] < x ...
     *
     * Because of the nature of interval 0, fClosures[0] has no
     * effect.

     */
    double*         fChoiceLimits;
    UBool*          fClosures;
    UnicodeString*  fChoiceFormats;
    int32_t         fCount;
};
 
inline UnicodeString&
ChoiceFormat::format(const Formattable& obj,
                     UnicodeString& appendTo,
                     UErrorCode& status) const {
    // Don't use Format:: - use immediate base class only,
    // in case immediate base modifies behavior later.
    return NumberFormat::format(obj, appendTo, status);
}

inline UnicodeString&
ChoiceFormat::format(double number,
                     UnicodeString& appendTo) const {
    return NumberFormat::format(number, appendTo);
}

inline UnicodeString&
ChoiceFormat::format(int32_t number,
                     UnicodeString& appendTo) const {
    return NumberFormat::format(number, appendTo);
}
U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _CHOICFMT
//eof
