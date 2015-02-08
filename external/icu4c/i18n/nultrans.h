#ifndef NULTRANS_H
#define NULTRANS_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/translit.h"

U_NAMESPACE_BEGIN

class NullTransliterator : public Transliterator {

public:

    /**
     * Constructs a transliterator.
     * @internal Use transliterator factory methods instead since this class will be removed in that release.
     */
    NullTransliterator();

    /**
     * Destructor.
     * @internal Use transliterator factory methods instead since this class will be removed in that release.
     */
    virtual ~NullTransliterator();

    /**
     * Transliterator API.
     * @internal Use transliterator factory methods instead since this class will be removed in that release.
     */
    virtual Transliterator* clone(void) const;

    /**
     * Implements {@link Transliterator#handleTransliterate}.
     * @internal Use transliterator factory methods instead since this class will be removed in that release.
     */
    virtual void handleTransliterate(Replaceable& text, UTransPosition& offset,
                                     UBool isIncremental) const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     */
    virtual UClassID getDynamicClassID() const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     */
    U_I18N_API static UClassID U_EXPORT2 getStaticClassID();

};

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

#endif
