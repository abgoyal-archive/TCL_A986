#ifndef USRCHIMP_H
#define USRCHIMP_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "unicode/ucol.h"
#include "unicode/ucoleitr.h"
#include "unicode/ubrk.h"

#define INITIAL_ARRAY_SIZE_       256
#define MAX_TABLE_SIZE_           257

struct USearch {
    // required since collation element iterator does not have a getText API
    const UChar              *text;
          int32_t             textLength; // exact length
          UBool               isOverlap;
          UBool               isCanonicalMatch;
          UBreakIterator	 *internalBreakIter;  //internal character breakiterator
          UBreakIterator     *breakIter;
    // value USEARCH_DONE is the default value
    // if we are not at the start of the text or the end of the text, 
    // depending on the iteration direction and matchedIndex is USEARCH_DONE 
    // it means that we can't find any more matches in that particular direction
          int32_t             matchedIndex; 
          int32_t             matchedLength;
          UBool               isForwardSearching;
          UBool               reset;
};

struct UPattern {
    const UChar              *text;
          int32_t             textLength; // exact length
          // length required for backwards ce comparison
          int32_t             CELength; 
          int32_t            *CE;
          int32_t             CEBuffer[INITIAL_ARRAY_SIZE_];
          int32_t             PCELength;
          int64_t            *PCE;
          int64_t             PCEBuffer[INITIAL_ARRAY_SIZE_];
          UBool               hasPrefixAccents;
          UBool               hasSuffixAccents;
          int16_t             defaultShiftSize;
          int16_t             shift[MAX_TABLE_SIZE_];
          int16_t             backShift[MAX_TABLE_SIZE_];
};

struct UStringSearch {
    struct USearch            *search;
    struct UPattern            pattern;
    const  UCollator          *collator;
    // positions within the collation element iterator is used to determine
    // if we are at the start of the text.
           UCollationElements *textIter;
    // utility collation element, used throughout program for temporary 
    // iteration.
           UCollationElements *utilIter;
           UBool               ownCollator;
           UCollationStrength  strength;
           uint32_t            ceMask;
           uint32_t            variableTop;
           UBool               toShift;
           UChar               canonicalPrefixAccents[INITIAL_ARRAY_SIZE_];
           UChar               canonicalSuffixAccents[INITIAL_ARRAY_SIZE_];
};

U_CFUNC
UBool usearch_handleNextExact(UStringSearch *strsrch, UErrorCode *status);

U_CFUNC
UBool usearch_handleNextCanonical(UStringSearch *strsrch, UErrorCode *status);

U_CFUNC
UBool usearch_handlePreviousExact(UStringSearch *strsrch, UErrorCode *status);

U_CFUNC
UBool usearch_handlePreviousCanonical(UStringSearch *strsrch, 
                                      UErrorCode    *status);

#endif /* #if !UCONFIG_NO_COLLATION */

#endif
