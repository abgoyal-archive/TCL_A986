
#ifndef USEARCH_H
#define USEARCH_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "unicode/ucol.h"
#include "unicode/ucoleitr.h"
#include "unicode/ubrk.h"


#define USEARCH_DONE -1

struct UStringSearch;
typedef struct UStringSearch UStringSearch;

typedef enum {
    /** Option for overlapping matches */
    USEARCH_OVERLAP,
    /** 
    Option for canonical matches. option 1 in header documentation.
    The default value will be USEARCH_OFF
    */
    USEARCH_CANONICAL_MATCH,
    USEARCH_ATTRIBUTE_COUNT
} USearchAttribute;

typedef enum {
    /** default value for any USearchAttribute */
    USEARCH_DEFAULT = -1,
    /** value for USEARCH_OVERLAP and USEARCH_CANONICAL_MATCH */
    USEARCH_OFF, 
    /** value for USEARCH_OVERLAP and USEARCH_CANONICAL_MATCH */
    USEARCH_ON,
    USEARCH_ATTRIBUTE_VALUE_COUNT
} USearchAttributeValue;

/* open and close ------------------------------------------------------ */

U_STABLE UStringSearch * U_EXPORT2 usearch_open(const UChar          *pattern, 
                                              int32_t         patternlength, 
                                        const UChar          *text, 
                                              int32_t         textlength,
                                        const char           *locale,
                                              UBreakIterator *breakiter,
                                              UErrorCode     *status);

U_STABLE UStringSearch * U_EXPORT2 usearch_openFromCollator(
                                         const UChar *pattern, 
                                               int32_t         patternlength,
                                         const UChar          *text, 
                                               int32_t         textlength,
                                         const UCollator      *collator,
                                               UBreakIterator *breakiter,
                                               UErrorCode     *status);

U_STABLE void U_EXPORT2 usearch_close(UStringSearch *searchiter);

/* get and set methods -------------------------------------------------- */

U_STABLE void U_EXPORT2 usearch_setOffset(UStringSearch *strsrch, 
                                        int32_t    position,
                                        UErrorCode    *status);

U_STABLE int32_t U_EXPORT2 usearch_getOffset(const UStringSearch *strsrch);
    
U_STABLE void U_EXPORT2 usearch_setAttribute(UStringSearch         *strsrch, 
                                           USearchAttribute       attribute,
                                           USearchAttributeValue  value,
                                           UErrorCode            *status);

U_STABLE USearchAttributeValue U_EXPORT2 usearch_getAttribute(
                                         const UStringSearch    *strsrch,
                                               USearchAttribute  attribute);

U_STABLE int32_t U_EXPORT2 usearch_getMatchedStart(
                                               const UStringSearch *strsrch);
    
U_STABLE int32_t U_EXPORT2 usearch_getMatchedLength(
                                               const UStringSearch *strsrch);

U_STABLE int32_t U_EXPORT2 usearch_getMatchedText(const UStringSearch *strsrch, 
                                            UChar         *result, 
                                            int32_t        resultCapacity, 
                                            UErrorCode    *status);

#if !UCONFIG_NO_BREAK_ITERATION

U_STABLE void U_EXPORT2 usearch_setBreakIterator(UStringSearch  *strsrch, 
                                               UBreakIterator *breakiter,
                                               UErrorCode     *status);

U_STABLE const UBreakIterator * U_EXPORT2 usearch_getBreakIterator(
                                              const UStringSearch *strsrch);
    
#endif
    
U_STABLE void U_EXPORT2 usearch_setText(      UStringSearch *strsrch, 
                                      const UChar         *text,
                                            int32_t        textlength,
                                            UErrorCode    *status);

U_STABLE const UChar * U_EXPORT2 usearch_getText(const UStringSearch *strsrch, 
                                               int32_t       *length);

U_STABLE UCollator * U_EXPORT2 usearch_getCollator(
                                               const UStringSearch *strsrch);

U_STABLE void U_EXPORT2 usearch_setCollator(      UStringSearch *strsrch, 
                                          const UCollator     *collator,
                                                UErrorCode    *status);

U_STABLE void U_EXPORT2 usearch_setPattern(      UStringSearch *strsrch, 
                                         const UChar         *pattern,
                                               int32_t        patternlength,
                                               UErrorCode    *status);

U_STABLE const UChar * U_EXPORT2 usearch_getPattern(
                                               const UStringSearch *strsrch, 
                                                     int32_t       *length);

/* methods ------------------------------------------------------------- */

U_STABLE int32_t U_EXPORT2 usearch_first(UStringSearch *strsrch, 
                                           UErrorCode    *status);

U_STABLE int32_t U_EXPORT2 usearch_following(UStringSearch *strsrch, 
                                               int32_t    position, 
                                               UErrorCode    *status);
    
U_STABLE int32_t U_EXPORT2 usearch_last(UStringSearch *strsrch, 
                                          UErrorCode    *status);

U_STABLE int32_t U_EXPORT2 usearch_preceding(UStringSearch *strsrch, 
                                               int32_t    position, 
                                               UErrorCode    *status);
    
U_STABLE int32_t U_EXPORT2 usearch_next(UStringSearch *strsrch, 
                                          UErrorCode    *status);

U_STABLE int32_t U_EXPORT2 usearch_previous(UStringSearch *strsrch, 
                                              UErrorCode    *status);
    
U_STABLE void U_EXPORT2 usearch_reset(UStringSearch *strsrch);

#endif /* #if !UCONFIG_NO_COLLATION */

#endif
