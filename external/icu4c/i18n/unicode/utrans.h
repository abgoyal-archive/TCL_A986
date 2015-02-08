
#ifndef UTRANS_H
#define UTRANS_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/localpointer.h"
#include "unicode/urep.h"
#include "unicode/parseerr.h"
#include "unicode/uenum.h"



typedef void* UTransliterator;

typedef enum UTransDirection {
    
    /**
     * UTRANS_FORWARD means from &lt;source&gt; to &lt;target&gt; for a
     * transliterator with ID &lt;source&gt;-&lt;target&gt;.  For a transliterator
     * opened using a rule, it means forward direction rules, e.g.,
     * "A > B".
     */
    UTRANS_FORWARD,

    /**
     * UTRANS_REVERSE means from &lt;target&gt; to &lt;source&gt; for a
     * transliterator with ID &lt;source&gt;-&lt;target&gt;.  For a transliterator
     * opened using a rule, it means reverse direction rules, e.g.,
     * "A < B".
     */
    UTRANS_REVERSE

} UTransDirection;

typedef struct UTransPosition {

    /**
     * Beginning index, inclusive, of the context to be considered for
     * a transliteration operation.  The transliterator will ignore
     * anything before this index.  INPUT/OUTPUT parameter: This parameter
     * is updated by a transliteration operation to reflect the maximum
     * amount of antecontext needed by a transliterator.
     * @stable ICU 2.4
     */
    int32_t contextStart;
    
    /**
     * Ending index, exclusive, of the context to be considered for a
     * transliteration operation.  The transliterator will ignore
     * anything at or after this index.  INPUT/OUTPUT parameter: This
     * parameter is updated to reflect changes in the length of the
     * text, but points to the same logical position in the text.
     * @stable ICU 2.4
     */
    int32_t contextLimit;
    
    /**
     * Beginning index, inclusive, of the text to be transliteratd.
     * INPUT/OUTPUT parameter: This parameter is advanced past
     * characters that have already been transliterated by a
     * transliteration operation.
     * @stable ICU 2.4
     */
    int32_t start;
    
    /**
     * Ending index, exclusive, of the text to be transliteratd.
     * INPUT/OUTPUT parameter: This parameter is updated to reflect
     * changes in the length of the text, but points to the same
     * logical position in the text.
     * @stable ICU 2.4
     */
    int32_t limit;

} UTransPosition;


U_STABLE UTransliterator* U_EXPORT2
utrans_openU(const UChar *id,
             int32_t idLength,
             UTransDirection dir,
             const UChar *rules,
             int32_t rulesLength,
             UParseError *parseError,
             UErrorCode *pErrorCode);

U_STABLE UTransliterator* U_EXPORT2 
utrans_openInverse(const UTransliterator* trans,
                   UErrorCode* status);

U_STABLE UTransliterator* U_EXPORT2 
utrans_clone(const UTransliterator* trans,
             UErrorCode* status);

U_STABLE void U_EXPORT2 
utrans_close(UTransliterator* trans);

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

U_DEFINE_LOCAL_OPEN_POINTER(LocalUTransliteratorPointer, UTransliterator, utrans_close);

U_NAMESPACE_END

#endif

U_STABLE const UChar * U_EXPORT2
utrans_getUnicodeID(const UTransliterator *trans,
                    int32_t *resultLength);

U_STABLE void U_EXPORT2 
utrans_register(UTransliterator* adoptedTrans,
                UErrorCode* status);

U_STABLE void U_EXPORT2
utrans_unregisterID(const UChar* id, int32_t idLength);

U_STABLE void U_EXPORT2 
utrans_setFilter(UTransliterator* trans,
                 const UChar* filterPattern,
                 int32_t filterPatternLen,
                 UErrorCode* status);

U_STABLE int32_t U_EXPORT2 
utrans_countAvailableIDs(void);

U_STABLE UEnumeration * U_EXPORT2
utrans_openIDs(UErrorCode *pErrorCode);


U_STABLE void U_EXPORT2 
utrans_trans(const UTransliterator* trans,
             UReplaceable* rep,
             UReplaceableCallbacks* repFunc,
             int32_t start,
             int32_t* limit,
             UErrorCode* status);

U_STABLE void U_EXPORT2 
utrans_transIncremental(const UTransliterator* trans,
                        UReplaceable* rep,
                        UReplaceableCallbacks* repFunc,
                        UTransPosition* pos,
                        UErrorCode* status);

U_STABLE void U_EXPORT2 
utrans_transUChars(const UTransliterator* trans,
                   UChar* text,
                   int32_t* textLength,
                   int32_t textCapacity,
                   int32_t start,
                   int32_t* limit,
                   UErrorCode* status);

U_STABLE void U_EXPORT2 
utrans_transIncrementalUChars(const UTransliterator* trans,
                              UChar* text,
                              int32_t* textLength,
                              int32_t textCapacity,
                              UTransPosition* pos,
                              UErrorCode* status);

/* deprecated API ----------------------------------------------------------- */

/* see utrans.h documentation for why these functions are deprecated */

U_DEPRECATED UTransliterator* U_EXPORT2 
utrans_open(const char* id,
            UTransDirection dir,
            const UChar* rules,         /* may be Null */
            int32_t rulesLength,        /* -1 if null-terminated */ 
            UParseError* parseError,    /* may be Null */
            UErrorCode* status);

U_DEPRECATED int32_t U_EXPORT2 
utrans_getID(const UTransliterator* trans,
             char* buf,
             int32_t bufCapacity);

U_DEPRECATED void U_EXPORT2 
utrans_unregister(const char* id);

U_DEPRECATED int32_t U_EXPORT2 
utrans_getAvailableID(int32_t index,
                      char* buf,
                      int32_t bufCapacity);

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

#endif
