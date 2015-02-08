#ifndef UNI2NAME_H
#define UNI2NAME_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/translit.h"

U_NAMESPACE_BEGIN

class UnicodeNameTransliterator : public Transliterator {

 public:

    /**
     * Constructs a transliterator.
     * @param adoptedFilter the filter to be adopted.
     */
    UnicodeNameTransliterator(UnicodeFilter* adoptedFilter = 0);

    /**
     * Destructor.
     */
    virtual ~UnicodeNameTransliterator();

    /**
     * Copy constructor.
     */
    UnicodeNameTransliterator(const UnicodeNameTransliterator&);

    /**
     * Transliterator API.
     */
    virtual Transliterator* clone(void) const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     */
    virtual UClassID getDynamicClassID() const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     */
    U_I18N_API static UClassID U_EXPORT2 getStaticClassID();

 protected:

    /**
     * Implements {@link Transliterator#handleTransliterate}.
     * @param text        the buffer holding transliterated and
     *                    untransliterated text
     * @param offset      the start and limit of the text, the position
     *                    of the cursor, and the start and limit of transliteration.
     * @param incremental if true, assume more text may be coming after
     *                    pos.contextLimit.  Otherwise, assume the text is complete.
     */
    virtual void handleTransliterate(Replaceable& text, UTransPosition& offset,
                                     UBool isIncremental) const;

private:
    /**
     * Assignment operator.
     */
    UnicodeNameTransliterator& operator=(const UnicodeNameTransliterator&);

};

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

#endif
