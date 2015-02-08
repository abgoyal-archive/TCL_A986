// *****************************************************************************
// This file was generated from the java source file Format.java
// *****************************************************************************

#ifndef FORMAT_H
#define FORMAT_H


#include "unicode/utypes.h"


#if !UCONFIG_NO_FORMATTING

#include "unicode/unistr.h"
#include "unicode/fmtable.h"
#include "unicode/fieldpos.h"
#include "unicode/parsepos.h"
#include "unicode/parseerr.h" 
#include "unicode/locid.h"

U_NAMESPACE_BEGIN

class U_I18N_API Format : public UObject {
public:

    /** Destructor
     * @stable ICU 2.4
     */
    virtual ~Format();

    /**
     * Return true if the given Format objects are semantically equal.
     * Objects of different subclasses are considered unequal.
     * @param other    the object to be compared with.
     * @return         Return true if the given Format objects are semantically equal.
     *                 Objects of different subclasses are considered unequal.
     * @stable ICU 2.0
     */
    virtual UBool operator==(const Format& other) const = 0;

    /**
     * Return true if the given Format objects are not semantically
     * equal.
     * @param other    the object to be compared with.
     * @return         Return true if the given Format objects are not semantically.
     * @stable ICU 2.0
     */
    UBool operator!=(const Format& other) const { return !operator==(other); }

    /**
     * Clone this object polymorphically.  The caller is responsible
     * for deleting the result when done.
     * @return    A copy of the object
     * @stable ICU 2.0
     */
    virtual Format* clone() const = 0;

    /**
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
     * Format an object to produce a string.  This is a pure virtual method which
     * subclasses must implement. This method allows polymorphic formatting
     * of Formattable objects. If a subclass of Format receives a Formattable
     * object type it doesn't handle (e.g., if a numeric Formattable is passed
     * to a DateFormat object) then it returns a failing UErrorCode.
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
                                  UErrorCode& status) const = 0;

    /**
     * Parse a string to produce an object.  This is a pure virtual
     * method which subclasses must implement.  This method allows
     * polymorphic parsing of strings into Formattable objects.
     * <P>
     * Before calling, set parse_pos.index to the offset you want to
     * start parsing at in the source.  After calling, parse_pos.index
     * is the end of the text you parsed.  If error occurs, index is
     * unchanged.
     * <P>
     * When parsing, leading whitespace is discarded (with successful
     * parse), while trailing whitespace is left as is.
     * <P>
     * Example:
     * <P>
     * Parsing "_12_xy" (where _ represents a space) for a number,
     * with index == 0 will result in the number 12, with
     * parse_pos.index updated to 3 (just before the second space).
     * Parsing a second time will result in a failing UErrorCode since
     * "xy" is not a number, and leave index at 3.
     * <P>
     * Subclasses will typically supply specific parse methods that
     * return different types of values. Since methods can't overload
     * on return types, these will typically be named "parse", while
     * this polymorphic method will always be called parseObject.  Any
     * parse method that does not take a parse_pos should set status
     * to an error value when no text in the required format is at the
     * start position.
     *
     * @param source    The string to be parsed into an object.
     * @param result    Formattable to be set to the parse result.
     *                  If parse fails, return contents are undefined.
     * @param parse_pos The position to start parsing at. Upon return
     *                  this param is set to the position after the
     *                  last character successfully parsed. If the
     *                  source is not parsed successfully, this param
     *                  will remain unchanged.
     * @stable ICU 2.0
     */
    virtual void parseObject(const UnicodeString& source,
                             Formattable& result,
                             ParsePosition& parse_pos) const = 0;

    /**
     * Parses a string to produce an object. This is a convenience method
     * which calls the pure virtual parseObject() method, and returns a
     * failure UErrorCode if the ParsePosition indicates failure.
     *
     * @param source    The string to be parsed into an object.
     * @param result    Formattable to be set to the parse result.
     *                  If parse fails, return contents are undefined.
     * @param status    Output param to be filled with success/failure
     *                  result code.
     * @stable ICU 2.0
     */
    void parseObject(const UnicodeString& source,
                     Formattable& result,
                     UErrorCode& status) const;

    /**
     * Returns a unique class ID POLYMORPHICALLY.  Pure virtual method.
     * This method is to implement a simple version of RTTI, since not all
     * C++ compilers support genuine RTTI.  Polymorphic operator==() and
     * clone() methods call this method.
     * Concrete subclasses of Format must implement getDynamicClassID()
     *
     * @return          The class ID for this object. All objects of a
     *                  given class have the same class ID.  Objects of
     *                  other classes have different class IDs.
     * @stable ICU 2.0
     */
    virtual UClassID getDynamicClassID() const = 0;

    /** Get the locale for this format object. You can choose between valid and actual locale.
     *  @param type type of the locale we're looking for (valid or actual) 
     *  @param status error code for the operation
     *  @return the locale
     *  @stable ICU 2.8
     */
    Locale getLocale(ULocDataLocaleType type, UErrorCode& status) const;

    /** Get the locale for this format object. You can choose between valid and actual locale.
     *  @param type type of the locale we're looking for (valid or actual) 
     *  @param status error code for the operation
     *  @return the locale
     *  @internal
     */
    const char* getLocaleID(ULocDataLocaleType type, UErrorCode &status) const;

 protected:
    /** @stable ICU 2.8 */
    void setLocaleIDs(const char* valid, const char* actual);

protected:
    /**
     * Default constructor for subclass use only.  Does nothing.
     * @stable ICU 2.0
     */
    Format();

    /**
     * @stable ICU 2.0
     */
    Format(const Format&); // Does nothing; for subclasses only

    /**
     * @stable ICU 2.0
     */
    Format& operator=(const Format&); // Does nothing; for subclasses

       
    /**
     * Simple function for initializing a UParseError from a UnicodeString.
     *
     * @param pattern The pattern to copy into the parseError
     * @param pos The position in pattern where the error occured
     * @param parseError The UParseError object to fill in
     * @stable ICU 2.4
     */
    static void syntaxError(const UnicodeString& pattern,
                            int32_t pos,
                            UParseError& parseError);

 private:
    char actualLocale[ULOC_FULLNAME_CAPACITY];
    char validLocale[ULOC_FULLNAME_CAPACITY];
};

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _FORMAT
//eof
