#ifndef UNIFILT_H
#define UNIFILT_H

#include "unicode/unifunct.h"
#include "unicode/unimatch.h"


U_NAMESPACE_BEGIN

#define U_ETHER ((UChar)0xFFFF)

class U_COMMON_API UnicodeFilter : public UnicodeFunctor, public UnicodeMatcher {

public:
    /**
     * Destructor
     * @stable ICU 2.0
     */
    virtual ~UnicodeFilter();

    /**
     * Returns <tt>true</tt> for characters that are in the selected
     * subset.  In other words, if a character is <b>to be
     * filtered</b>, then <tt>contains()</tt> returns
     * <b><tt>false</tt></b>.
     * @stable ICU 2.0
     */
    virtual UBool contains(UChar32 c) const = 0;

    /**
     * UnicodeFunctor API.  Cast 'this' to a UnicodeMatcher* pointer
     * and return the pointer.
     * @stable ICU 2.4
     */
    virtual UnicodeMatcher* toMatcher() const;

    /**
     * Implement UnicodeMatcher API.
     * @stable ICU 2.4
     */
    virtual UMatchDegree matches(const Replaceable& text,
                                 int32_t& offset,
                                 int32_t limit,
                                 UBool incremental);

    /**
     * UnicodeFunctor API.  Nothing to do.
     * @stable ICU 2.4
     */
    virtual void setData(const TransliterationRuleData*);

    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     *
     * @stable ICU 2.2
     */
    virtual UClassID getDynamicClassID() const = 0;

    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     *
     * @stable ICU 2.2
     */
    static UClassID U_EXPORT2 getStaticClassID();

protected:

    /*
     * Since this class has pure virtual functions,
     * a constructor can't be used.
     * @stable ICU 2.0
     */
/*    UnicodeFilter();*/
};

/*inline UnicodeFilter::UnicodeFilter() {}*/

U_NAMESPACE_END

#endif
