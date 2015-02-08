
#ifndef RBNF_H
#define RBNF_H

#include "unicode/utypes.h"


#if UCONFIG_NO_FORMATTING
#define U_HAVE_RBNF 0
#else
#define U_HAVE_RBNF 1

#include "unicode/coll.h"
#include "unicode/dcfmtsym.h"
#include "unicode/fmtable.h"
#include "unicode/locid.h"
#include "unicode/numfmt.h"
#include "unicode/unistr.h"
#include "unicode/strenum.h"

U_NAMESPACE_BEGIN

class NFRuleSet;
class LocalizationInfo;

enum URBNFRuleSetTag {
    URBNF_SPELLOUT,
    URBNF_ORDINAL,
    URBNF_DURATION,
    URBNF_NUMBERING_SYSTEM,
    URBNF_COUNT
};

#if UCONFIG_NO_COLLATION
class Collator;
#endif

class U_I18N_API RuleBasedNumberFormat : public NumberFormat {
public:

  //-----------------------------------------------------------------------
  // constructors
  //-----------------------------------------------------------------------

    /**
     * Creates a RuleBasedNumberFormat that behaves according to the description
     * passed in.  The formatter uses the default locale.
     * @param rules A description of the formatter's desired behavior.
     * See the class documentation for a complete explanation of the description
     * syntax.
     * @param perror The parse error if an error was encountered.
     * @param status The status indicating whether the constructor succeeded.
     * @stable ICU 3.2
     */
    RuleBasedNumberFormat(const UnicodeString& rules, UParseError& perror, UErrorCode& status);

    /**
     * Creates a RuleBasedNumberFormat that behaves according to the description
     * passed in.  The formatter uses the default locale.  
     * <p>
     * The localizations data provides information about the public
     * rule sets and their localized display names for different
     * locales. The first element in the list is an array of the names
     * of the public rule sets.  The first element in this array is
     * the initial default ruleset.  The remaining elements in the
     * list are arrays of localizations of the names of the public
     * rule sets.  Each of these is one longer than the initial array,
     * with the first String being the ULocale ID, and the remaining
     * Strings being the localizations of the rule set names, in the
     * same order as the initial array.  Arrays are NULL-terminated.
     * @param rules A description of the formatter's desired behavior.
     * See the class documentation for a complete explanation of the description
     * syntax.
     * @param localizations the localization information.
     * names in the description.  These will be copied by the constructor.
     * @param perror The parse error if an error was encountered.
     * @param status The status indicating whether the constructor succeeded.
     * @stable ICU 3.2
     */
    RuleBasedNumberFormat(const UnicodeString& rules, const UnicodeString& localizations,
                        UParseError& perror, UErrorCode& status);

  /**
   * Creates a RuleBasedNumberFormat that behaves according to the rules
   * passed in.  The formatter uses the specified locale to determine the
   * characters to use when formatting numerals, and to define equivalences
   * for lenient parsing.
   * @param rules The formatter rules.
   * See the class documentation for a complete explanation of the rule
   * syntax.
   * @param locale A locale that governs which characters are used for
   * formatting values in numerals and which characters are equivalent in
   * lenient parsing.
   * @param perror The parse error if an error was encountered.
   * @param status The status indicating whether the constructor succeeded.
   * @stable ICU 2.0
   */
  RuleBasedNumberFormat(const UnicodeString& rules, const Locale& locale,
                        UParseError& perror, UErrorCode& status);

    /**
     * Creates a RuleBasedNumberFormat that behaves according to the description
     * passed in.  The formatter uses the default locale.  
     * <p>
     * The localizations data provides information about the public
     * rule sets and their localized display names for different
     * locales. The first element in the list is an array of the names
     * of the public rule sets.  The first element in this array is
     * the initial default ruleset.  The remaining elements in the
     * list are arrays of localizations of the names of the public
     * rule sets.  Each of these is one longer than the initial array,
     * with the first String being the ULocale ID, and the remaining
     * Strings being the localizations of the rule set names, in the
     * same order as the initial array.  Arrays are NULL-terminated.
     * @param rules A description of the formatter's desired behavior.
     * See the class documentation for a complete explanation of the description
     * syntax.
     * @param localizations a list of localizations for the rule set
     * names in the description.  These will be copied by the constructor.
     * @param locale A locale that governs which characters are used for
     * formatting values in numerals and which characters are equivalent in
     * lenient parsing.
     * @param perror The parse error if an error was encountered.
     * @param status The status indicating whether the constructor succeeded.
     * @stable ICU 3.2
     */
    RuleBasedNumberFormat(const UnicodeString& rules, const UnicodeString& localizations,
                        const Locale& locale, UParseError& perror, UErrorCode& status);

  /**
   * Creates a RuleBasedNumberFormat from a predefined ruleset.  The selector
   * code choosed among three possible predefined formats: spellout, ordinal,
   * and duration.
   * @param tag A selector code specifying which kind of formatter to create for that
   * locale.  There are four legal values: URBNF_SPELLOUT, which creates a formatter that
   * spells out a value in words in the desired language, URBNF_ORDINAL, which attaches
   * an ordinal suffix from the desired language to the end of a number (e.g. "123rd"),
   * URBNF_DURATION, which formats a duration in seconds as hours, minutes, and seconds,
   * and URBNF_NUMBERING_SYSTEM, which is used to invoke rules for alternate numbering
   * systems such as the Hebrew numbering system, or for Roman Numerals, etc.
   * @param locale The locale for the formatter.
   * @param status The status indicating whether the constructor succeeded.
   * @stable ICU 2.0
   */
  RuleBasedNumberFormat(URBNFRuleSetTag tag, const Locale& locale, UErrorCode& status);

  //-----------------------------------------------------------------------
  // boilerplate
  //-----------------------------------------------------------------------

  /**
   * Copy constructor
   * @param rhs    the object to be copied from.
   * @stable ICU 2.6
   */
  RuleBasedNumberFormat(const RuleBasedNumberFormat& rhs);

  /**
   * Assignment operator
   * @param rhs    the object to be copied from.
   * @stable ICU 2.6
   */
  RuleBasedNumberFormat& operator=(const RuleBasedNumberFormat& rhs);

  /**
   * Release memory allocated for a RuleBasedNumberFormat when you are finished with it.
   * @stable ICU 2.6
   */
  virtual ~RuleBasedNumberFormat();

  /**
   * Clone this object polymorphically.  The caller is responsible
   * for deleting the result when done.
   * @return  A copy of the object.
   * @stable ICU 2.6
   */
  virtual Format* clone(void) const;

  /**
   * Return true if the given Format objects are semantically equal.
   * Objects of different subclasses are considered unequal.
   * @param other    the object to be compared with.
   * @return        true if the given Format objects are semantically equal.
   * @stable ICU 2.6
   */
  virtual UBool operator==(const Format& other) const;

//-----------------------------------------------------------------------
// public API functions
//-----------------------------------------------------------------------

  /**
   * return the rules that were provided to the RuleBasedNumberFormat.
   * @return the result String that was passed in
   * @stable ICU 2.0
   */
  virtual UnicodeString getRules() const;

  /**
   * Return the number of public rule set names.
   * @return the number of public rule set names.
   * @stable ICU 2.0
   */
  virtual int32_t getNumberOfRuleSetNames() const;

  /**
   * Return the name of the index'th public ruleSet.  If index is not valid,
   * the function returns null.
   * @param index the index of the ruleset
   * @return the name of the index'th public ruleSet.
   * @stable ICU 2.0
   */
  virtual UnicodeString getRuleSetName(int32_t index) const;

  /**
   * Return the number of locales for which we have localized rule set display names.
   * @return the number of locales for which we have localized rule set display names.
   * @stable ICU 3.2
   */
  virtual int32_t getNumberOfRuleSetDisplayNameLocales(void) const;

  /**
   * Return the index'th display name locale.
   * @param index the index of the locale
   * @param status set to a failure code when this function fails
   * @return the locale
   * @see #getNumberOfRuleSetDisplayNameLocales
   * @stable ICU 3.2
   */
  virtual Locale getRuleSetDisplayNameLocale(int32_t index, UErrorCode& status) const;

    /**
     * Return the rule set display names for the provided locale.  These are in the same order
     * as those returned by getRuleSetName.  The locale is matched against the locales for
     * which there is display name data, using normal fallback rules.  If no locale matches, 
     * the default display names are returned.  (These are the internal rule set names minus
     * the leading '%'.)
     * @param index the index of the rule set
     * @param locale the locale (returned by getRuleSetDisplayNameLocales) for which the localized
     * display name is desired
     * @return the display name for the given index, which might be bogus if there is an error
     * @see #getRuleSetName
     * @stable ICU 3.2
     */
  virtual UnicodeString getRuleSetDisplayName(int32_t index, 
                          const Locale& locale = Locale::getDefault());

    /**
     * Return the rule set display name for the provided rule set and locale.  
     * The locale is matched against the locales for which there is display name data, using
     * normal fallback rules.  If no locale matches, the default display name is returned.
     * @return the display name for the rule set
     * @stable ICU 3.2
     * @see #getRuleSetDisplayName
     */
  virtual UnicodeString getRuleSetDisplayName(const UnicodeString& ruleSetName, 
                          const Locale& locale = Locale::getDefault());

  /**
   * Formats the specified 32-bit number using the default ruleset.
   * @param number The number to format.
   * @param toAppendTo the string that will hold the (appended) result
   * @param pos the fieldposition
   * @return A textual representation of the number.
   * @stable ICU 2.0
   */
  virtual UnicodeString& format(int32_t number,
                                UnicodeString& toAppendTo,
                                FieldPosition& pos) const;

  /**
   * Formats the specified 64-bit number using the default ruleset.
   * @param number The number to format.
   * @param toAppendTo the string that will hold the (appended) result
   * @param pos the fieldposition
   * @return A textual representation of the number.
   * @stable ICU 2.1
   */
  virtual UnicodeString& format(int64_t number,
                                UnicodeString& toAppendTo,
                                FieldPosition& pos) const;
  /**
   * Formats the specified number using the default ruleset.
   * @param number The number to format.
   * @param toAppendTo the string that will hold the (appended) result
   * @param pos the fieldposition
   * @return A textual representation of the number.
   * @stable ICU 2.0
   */
  virtual UnicodeString& format(double number,
                                UnicodeString& toAppendTo,
                                FieldPosition& pos) const;

  /**
   * Formats the specified number using the named ruleset.
   * @param number The number to format.
   * @param ruleSetName The name of the rule set to format the number with.
   * This must be the name of a valid public rule set for this formatter.
   * @param toAppendTo the string that will hold the (appended) result
   * @param pos the fieldposition
   * @param status the status
   * @return A textual representation of the number.
   * @stable ICU 2.0
   */
  virtual UnicodeString& format(int32_t number,
                                const UnicodeString& ruleSetName,
                                UnicodeString& toAppendTo,
                                FieldPosition& pos,
                                UErrorCode& status) const;
  /**
   * Formats the specified 64-bit number using the named ruleset.
   * @param number The number to format.
   * @param ruleSetName The name of the rule set to format the number with.
   * This must be the name of a valid public rule set for this formatter.
   * @param toAppendTo the string that will hold the (appended) result
   * @param pos the fieldposition
   * @param status the status
   * @return A textual representation of the number.
   * @stable ICU 2.1
   */
  virtual UnicodeString& format(int64_t number,
                                const UnicodeString& ruleSetName,
                                UnicodeString& toAppendTo,
                                FieldPosition& pos,
                                UErrorCode& status) const;
  /**
   * Formats the specified number using the named ruleset.
   * @param number The number to format.
   * @param ruleSetName The name of the rule set to format the number with.
   * This must be the name of a valid public rule set for this formatter.
   * @param toAppendTo the string that will hold the (appended) result
   * @param pos the fieldposition
   * @param status the status
   * @return A textual representation of the number.
   * @stable ICU 2.0
   */
  virtual UnicodeString& format(double number,
                                const UnicodeString& ruleSetName,
                                UnicodeString& toAppendTo,
                                FieldPosition& pos,
                                UErrorCode& status) const;

  /**
   * Formats the specified number using the default ruleset.
   * @param obj The number to format.
   * @param toAppendTo the string that will hold the (appended) result
   * @param pos the fieldposition
   * @param status the status
   * @return A textual representation of the number.
   * @stable ICU 2.0
   */
  virtual UnicodeString& format(const Formattable& obj,
                                UnicodeString& toAppendTo,
                                FieldPosition& pos,
                                UErrorCode& status) const;
  /**
   * Redeclared Format method.
   * @param obj    the object to be formatted.
   * @param result Output param which will receive the formatted string.
   * @param status Output param set to success/failure code
   * @return       A reference to 'result'.
   * @stable ICU 2.0
   */
  UnicodeString& format(const Formattable& obj,
                        UnicodeString& result,
                        UErrorCode& status) const;

  /**
   * Redeclared NumberFormat method.
   * @param number    the double value to be formatted.
   * @param output    Output param which will receive the formatted string.
   * @return          A reference to 'output'.
   * @stable ICU 2.0
   */
   UnicodeString& format(double number,
                         UnicodeString& output) const;

  /**
   * Redeclared NumberFormat method.
   * @param number    the long value to be formatted.
   * @param output    Output param which will receive the formatted string.
   * @return          A reference to 'output'.
   * @stable ICU 2.0
   */
   UnicodeString& format(int32_t number,
                         UnicodeString& output) const;

  /**
   * Parses the specfied string, beginning at the specified position, according
   * to this formatter's rules.  This will match the string against all of the
   * formatter's public rule sets and return the value corresponding to the longest
   * parseable substring.  This function's behavior is affected by the lenient
   * parse mode.
   * @param text The string to parse
   * @param result the result of the parse, either a double or a long.
   * @param parsePosition On entry, contains the position of the first character
   * in "text" to examine.  On exit, has been updated to contain the position
   * of the first character in "text" that wasn't consumed by the parse.
   * @see #setLenient
   * @stable ICU 2.0
   */
  virtual void parse(const UnicodeString& text,
                     Formattable& result,
                     ParsePosition& parsePosition) const;


  /**
   * Redeclared Format method.
   * @param text   The string to parse
   * @param result the result of the parse, either a double or a long.
   * @param status Output param set to failure code when a problem occurs.
   * @stable ICU 2.0
   */
  virtual inline void parse(const UnicodeString& text,
                      Formattable& result,
                      UErrorCode& status) const;

#if !UCONFIG_NO_COLLATION

  /**
   * Turns lenient parse mode on and off.
   *
   * When in lenient parse mode, the formatter uses a Collator for parsing the text.
   * Only primary differences are treated as significant.  This means that case
   * differences, accent differences, alternate spellings of the same letter
   * (e.g., ae and a-umlaut in German), ignorable characters, etc. are ignored in
   * matching the text.  In many cases, numerals will be accepted in place of words
   * or phrases as well.
   *
   * For example, all of the following will correctly parse as 255 in English in
   * lenient-parse mode:
   * <br>"two hundred fifty-five"
   * <br>"two hundred fifty five"
   * <br>"TWO HUNDRED FIFTY-FIVE"
   * <br>"twohundredfiftyfive"
   * <br>"2 hundred fifty-5"
   *
   * The Collator used is determined by the locale that was
   * passed to this object on construction.  The description passed to this object
   * on construction may supply additional collation rules that are appended to the
   * end of the default collator for the locale, enabling additional equivalences
   * (such as adding more ignorable characters or permitting spelled-out version of
   * symbols; see the demo program for examples).
   *
   * It's important to emphasize that even strict parsing is relatively lenient: it
   * will accept some text that it won't produce as output.  In English, for example,
   * it will correctly parse "two hundred zero" and "fifteen hundred".
   *
   * @param enabled If true, turns lenient-parse mode on; if false, turns it off.
   * @see RuleBasedCollator
   * @stable ICU 2.0
   */
  virtual void setLenient(UBool enabled);

  /**
   * Returns true if lenient-parse mode is turned on.  Lenient parsing is off
   * by default.
   * @return true if lenient-parse mode is turned on.
   * @see #setLenient
   * @stable ICU 2.0
   */
  virtual inline UBool isLenient(void) const;

#endif

  /**
   * Override the default rule set to use.  If ruleSetName is null, reset
   * to the initial default rule set.  If the rule set is not a public rule set name,
   * U_ILLEGAL_ARGUMENT_ERROR is returned in status.
   * @param ruleSetName the name of the rule set, or null to reset the initial default.
   * @param status set to failure code when a problem occurs.
   * @stable ICU 2.6
   */
  virtual void setDefaultRuleSet(const UnicodeString& ruleSetName, UErrorCode& status);

  /**
   * Return the name of the current default rule set.  If the current rule set is
   * not public, returns a bogus (and empty) UnicodeString.
   * @return the name of the current default rule set
   * @stable ICU 3.0
   */
  virtual UnicodeString getDefaultRuleSetName() const;

public:
    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     *
     * @stable ICU 2.8
     */
    static UClassID U_EXPORT2 getStaticClassID(void);

    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     *
     * @stable ICU 2.8
     */
    virtual UClassID getDynamicClassID(void) const;

private:
    RuleBasedNumberFormat(); // default constructor not implemented

    // this will ref the localizations if they are not NULL
    // caller must deref to get adoption 
    RuleBasedNumberFormat(const UnicodeString& description, LocalizationInfo* localizations, 
              const Locale& locale, UParseError& perror, UErrorCode& status);

    void init(const UnicodeString& rules, LocalizationInfo* localizations, UParseError& perror, UErrorCode& status);
    void dispose();
    void stripWhitespace(UnicodeString& src);
    void initDefaultRuleSet();
    void format(double number, NFRuleSet& ruleSet);
    NFRuleSet* findRuleSet(const UnicodeString& name, UErrorCode& status) const;

    /* friend access */
    friend class NFSubstitution;
    friend class NFRule;
    friend class FractionalPartSubstitution;

    inline NFRuleSet * getDefaultRuleSet() const;
    Collator * getCollator() const;
    DecimalFormatSymbols * getDecimalFormatSymbols() const;

private:
    NFRuleSet **ruleSets;
    NFRuleSet *defaultRuleSet;
    Locale locale;
    Collator* collator;
    DecimalFormatSymbols* decimalFormatSymbols;
    UBool lenient;
    UnicodeString* lenientParseRules;
    LocalizationInfo* localizations;

    // Temporary workaround - when noParse is true, do noting in parse.
    // TODO: We need a real fix - see #6895/#6896
    UBool noParse;
};

// ---------------

inline UnicodeString&
RuleBasedNumberFormat::format(const Formattable& obj,
                              UnicodeString& result,
                              UErrorCode& status) const
{
    // Don't use Format:: - use immediate base class only,
    // in case immediate base modifies behavior later.
    // dlf - the above comment is bogus, if there were a reason to modify
    // it, it would be virtual, and there's no reason because it is
    // a one-line macro in NumberFormat anyway, just like this one.
    return NumberFormat::format(obj, result, status);
}

inline UnicodeString&
RuleBasedNumberFormat::format(double number, UnicodeString& output) const {
    FieldPosition pos(0);
    return format(number, output, pos);
}

inline UnicodeString&
RuleBasedNumberFormat::format(int32_t number, UnicodeString& output) const {
    FieldPosition pos(0);
    return format(number, output, pos);
}

inline void
RuleBasedNumberFormat::parse(const UnicodeString& text, Formattable& result, UErrorCode& status) const
{
    NumberFormat::parse(text, result, status);
}

#if !UCONFIG_NO_COLLATION

inline UBool
RuleBasedNumberFormat::isLenient(void) const {
    return lenient;
}

#endif

inline NFRuleSet*
RuleBasedNumberFormat::getDefaultRuleSet() const {
    return defaultRuleSet;
}

U_NAMESPACE_END

/* U_HAVE_RBNF */
#endif

/* RBNF_H */
#endif
