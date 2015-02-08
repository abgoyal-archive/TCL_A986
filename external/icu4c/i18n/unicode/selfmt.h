
#ifndef SELFMT
#define SELFMT

#include "unicode/utypes.h"
#include "unicode/numfmt.h"


#if !UCONFIG_NO_FORMATTING

U_NAMESPACE_BEGIN

class Hashtable;


class U_I18N_API SelectFormat : public Format {
public:

    /**
     * Creates a new <code>SelectFormat</code> for a given pattern string.
     * @param  pattern the pattern for this <code>SelectFormat</code>.
     *                 errors are returned to status if the pattern is invalid.
     * @param status   output param set to success/failure code on exit, which
     *                 must not indicate a failure before the function call.
     * @draft ICU 4.4
     */
    SelectFormat(const UnicodeString& pattern, UErrorCode& status);

    /**
     * copy constructor.
     * @draft ICU 4.4
     */
    SelectFormat(const SelectFormat& other);

    /**
     * Destructor.
     * @draft ICU 4.4
     */
    virtual ~SelectFormat();

    /**
     * Sets the pattern used by this select format.
     * for the keyword rules.
     * Patterns and their interpretation are specified in the class description.
     *
     * @param pattern the pattern for this select format
     *                errors are returned to status if the pattern is invalid.
     * @param status  output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @draft ICU 4.4
     */
    void applyPattern(const UnicodeString& pattern, UErrorCode& status);


    using Format::format;

    /**
     * Selects the phrase for  the given keyword
     *
     * @param keyword  The keyword that is used to select an alternative.
     * @param appendTo output parameter to receive result.
     *                 result is appended to existing contents.
     * @param pos      On input: an alignment field, if desired.
     *                 On output: the offsets of the alignment field.
     * @param status  output param set to success/failure code on exit, which
     *                 must not indicate a failure before the function call.
     * @return         Reference to 'appendTo' parameter.
     * @draft ICU 4.4
     */
    UnicodeString& format(const UnicodeString& keyword,
                            UnicodeString& appendTo,
                            FieldPosition& pos,
                            UErrorCode& status) const;

    /**
     * Assignment operator
     *
     * @param other    the SelectFormat object to copy from.
     * @draft ICU 4.4
     */
    SelectFormat& operator=(const SelectFormat& other);

    /**
     * Return true if another object is semantically equal to this one.
     *
     * @param other    the SelectFormat object to be compared with.
     * @return         true if other is semantically equal to this.
     * @draft ICU 4.4
     */
    virtual UBool operator==(const Format& other) const;

    /**
     * Return true if another object is semantically unequal to this one.
     *
     * @param other    the SelectFormat object to be compared with.
     * @return         true if other is semantically unequal to this.
     * @draft ICU 4.4
     */
    virtual UBool operator!=(const Format& other) const;

    /**
     * Clones this Format object polymorphically.  The caller owns the
     * result and should delete it when done.
     * @draft ICU 4.4
     */
    virtual Format* clone(void) const;

    /**
     * Format an object to produce a string.
     * This method handles keyword strings.
     * If the Formattable object is not a <code>UnicodeString</code>,
     * then it returns a failing UErrorCode.
     *
     * @param obj       A keyword string that is used to select an alternative.
     * @param appendTo  output parameter to receive result.
     *                  Result is appended to existing contents.
     * @param pos       On input: an alignment field, if desired.
     *                  On output: the offsets of the alignment field.
     * @param status    output param filled with success/failure status.
     * @return          Reference to 'appendTo' parameter.
     * @draft ICU 4.4
     */
    UnicodeString& format(const Formattable& obj,
                         UnicodeString& appendTo,
                         FieldPosition& pos,
                         UErrorCode& status) const;

    /**
     * Returns the pattern from applyPattern() or constructor.
     *
     * @param  appendTo  output parameter to receive result.
     *                  Result is appended to existing contents.
     * @return the UnicodeString with inserted pattern.
     * @draft ICU 4.4
     */
    UnicodeString& toPattern(UnicodeString& appendTo);

    /**
     * This method is not yet supported by <code>SelectFormat</code>.
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
     * @param source     The string to be parsed into an object.
     * @param result     Formattable to be set to the parse result.
     *     If parse fails, return contents are undefined.
     * @param parse_pos The position to start parsing at. Upon return
     *     this param is set to the position after the
     *     last character successfully parsed. If the
     *     source is not parsed successfully, this param
     *     will remain unchanged.
     * @draft ICU 4.4
     */
    virtual void parseObject(const UnicodeString& source,
                            Formattable& result,
                            ParsePosition& parse_pos) const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     * @draft ICU 4.4
     */
    static UClassID U_EXPORT2 getStaticClassID(void);

    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     * @draft ICU 4.4
     */
    virtual UClassID getDynamicClassID() const;

private:
    typedef enum classesForSelectFormat{
        tStartKeyword,
        tContinueKeyword,
        tLeftBrace,
        tRightBrace,
        tSpace,
        tOther
    }CharacterClass;

    UnicodeString pattern;
    //Hash to store the keyword, phrase pairs.
    Hashtable  *parsedValuesHash;

    SelectFormat();   // default constructor not implemented.
    void init(UErrorCode& status);
    //For the applyPattern , classifies char.s in one of the characterClass.
    CharacterClass classifyCharacter(UChar ch) const;
    //Checks if the "other" keyword is present in pattern.
    UBool checkSufficientDefinition();
    //Checks if the keyword passed is valid.
    UBool checkValidKeyword(const UnicodeString& argKeyword) const;
    void parsingFailure();
    void copyHashtable(Hashtable *other, UErrorCode& status);
};

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _SELFMT
//eof
