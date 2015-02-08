
#ifndef PLURRULE
#define PLURRULE

#include "unicode/utypes.h"


#if !UCONFIG_NO_FORMATTING

#include "unicode/format.h"

U_NAMESPACE_BEGIN

class Hashtable;
class RuleChain;
class RuleParser;

class U_I18N_API PluralRules : public UObject {
public:

    /**
     * Constructor.
     * @param status  Output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     *
     * @stable ICU 4.0
     */
    PluralRules(UErrorCode& status);

    /**
     * Copy constructor.
     * @stable ICU 4.0
     */
    PluralRules(const PluralRules& other);

    /**
     * Destructor.
     * @stable ICU 4.0
     */
    virtual ~PluralRules();

    /**
     * Clone
     * @stable ICU 4.0
     */
    PluralRules* clone() const;

    /**
      * Assignment operator.
      * @stable ICU 4.0
      */
    PluralRules& operator=(const PluralRules&);

    /**
     * Creates a PluralRules from a description if it is parsable, otherwise
     * returns null.
     *
     * @param description rule description
     * @param status      Output param set to success/failure code on exit, which
     *                    must not indicate a failure before the function call.
     * @return            new PluralRules pointer. NULL if there is an error.
     * @stable ICU 4.0
     */
    static PluralRules* U_EXPORT2 createRules(const UnicodeString& description,
                                              UErrorCode& status);

    /**
     * The default rules that accept any number.
     *
     * @param status  Output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @return        new PluralRules pointer. NULL if there is an error.
     * @stable ICU 4.0
     */
    static PluralRules* U_EXPORT2 createDefaultRules(UErrorCode& status);

    /**
     * Provides access to the predefined <code>PluralRules</code> for a given
     * locale.
     *
     * @param locale  The locale for which a <code>PluralRules</code> object is
     *                returned.
     * @param status  Output param set to success/failure code on exit, which
     *                must not indicate a failure before the function call.
     * @return        The predefined <code>PluralRules</code> object pointer for
     *                this locale. If there's no predefined rules for this locale,
     *                the rules for the closest parent in the locale hierarchy
     *                that has one will  be returned.  The final fallback always
     *                returns the default 'other' rules.
     * @stable ICU 4.0
     */
    static PluralRules* U_EXPORT2 forLocale(const Locale& locale, UErrorCode& status);
    
    /**
     * Given a number, returns the keyword of the first rule that applies to
     * the number.  This function can be used with isKeyword* functions to
     * determine the keyword for default plural rules.
     *
     * @param number  The number for which the rule has to be determined.
     * @return        The keyword of the selected rule.
     * @stable ICU 4.0
     */
    UnicodeString select(int32_t number) const;
    
    /**
     * Given a number, returns the keyword of the first rule that applies to
     * the number.  This function can be used with isKeyword* functions to
     * determine the keyword for default plural rules.
     *
     * @param number  The number for which the rule has to be determined.
     * @return        The keyword of the selected rule.
     * @stable ICU 4.0
     */
    UnicodeString select(double number) const;

    /**
     * Returns a list of all rule keywords used in this <code>PluralRules</code>
     * object.  The rule 'other' is always present by default.
     *
     * @param status Output param set to success/failure code on exit, which
     *               must not indicate a failure before the function call.
     * @return       StringEnumeration with the keywords.
     *               The caller must delete the object.
     * @stable ICU 4.0
     */
    StringEnumeration* getKeywords(UErrorCode& status) const;

    /**
     * Returns TRUE if the given keyword is defined in this
     * <code>PluralRules</code> object.
     *
     * @param keyword  the input keyword.
     * @return         TRUE if the input keyword is defined.
     *                 Otherwise, return FALSE.
     * @stable ICU 4.0
     */
    UBool isKeyword(const UnicodeString& keyword) const;


    /**
     * Returns keyword for default plural form.
     *
     * @return         keyword for default plural form.
     * @internal 4.0
     * @stable ICU 4.0
     */
    UnicodeString getKeywordOther() const;

    /**
     * Compares the equality of two PluralRules objects.
     *
     * @param other The other PluralRules object to be compared with.
     * @return      True if the given PluralRules is the same as this
     *              PluralRules; false otherwise.
     * @stable ICU 4.0
     */
    virtual UBool operator==(const PluralRules& other) const;

    /**
     * Compares the inequality of two PluralRules objects.
     *
     * @param other The PluralRules object to be compared with.
     * @return      True if the given PluralRules is not the same as this
     *              PluralRules; false otherwise.
     * @stable ICU 4.0
     */
    UBool operator!=(const PluralRules& other) const  {return !operator==(other);}


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
    Hashtable       *fLocaleStringsHash;
    UnicodeString   mLocaleName;
    RuleChain       *mRules;
    RuleParser      *mParser;

    PluralRules();   // default constructor not implemented
    int32_t getRepeatLimit() const;
    void parseDescription(UnicodeString& ruleData, RuleChain& rules, UErrorCode &status);
    void getNextLocale(const UnicodeString& localeData, int32_t* curIndex, UnicodeString& localeName);
    void addRules(RuleChain& rules);
    int32_t getNumberValue(const UnicodeString& token) const;
    UnicodeString getRuleFromResource(const Locale& locale, UErrorCode& status);

};

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _PLURRULE
//eof
