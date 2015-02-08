



#ifndef __USET_H__
#define __USET_H__

#include "unicode/utypes.h"
#include "unicode/uchar.h"

#ifndef UCNV_H
struct USet;
typedef struct USet USet;
#endif

enum {
    /**
     * Ignore white space within patterns unless quoted or escaped.
     * @stable ICU 2.4
     */
    USET_IGNORE_SPACE = 1,  

    /**
     * Enable case insensitive matching.  E.g., "[ab]" with this flag
     * will match 'a', 'A', 'b', and 'B'.  "[^ab]" with this flag will
     * match all except 'a', 'A', 'b', and 'B'. This performs a full
     * closure over case mappings, e.g. U+017F for s.
     * @stable ICU 2.4
     */
    USET_CASE_INSENSITIVE = 2,  

    /**
     * Bitmask for UnicodeSet::closeOver() indicating letter case.
     * This may be ORed together with other selectors.
     * @internal
     */
    USET_CASE = 2,

    /**
     * Enable case insensitive matching.  E.g., "[ab]" with this flag
     * will match 'a', 'A', 'b', and 'B'.  "[^ab]" with this flag will
     * match all except 'a', 'A', 'b', and 'B'. This adds the lower-,
     * title-, and uppercase mappings as well as the case folding
     * of each existing element in the set.
     * @draft ICU 3.2
     */
    USET_ADD_CASE_MAPPINGS = 4,

    /**
     * Enough for any single-code point set
     * @internal
     */
    USET_SERIALIZED_STATIC_ARRAY_CAPACITY=8
};

typedef struct USerializedSet {
    /**
     * The serialized Unicode Set.
     * @stable ICU 2.4
     */
    const uint16_t *array;
    /**
     * The length of the array that contains BMP characters.
     * @stable ICU 2.4
     */
    int32_t bmpLength;
    /**
     * The total length of the array.
     * @stable ICU 2.4
     */
    int32_t length;
    /**
     * A small buffer for the array to reduce memory allocations.
     * @stable ICU 2.4
     */
    uint16_t staticArray[USET_SERIALIZED_STATIC_ARRAY_CAPACITY];
} USerializedSet;


U_STABLE USet* U_EXPORT2
uset_open(UChar32 start, UChar32 end);

U_STABLE USet* U_EXPORT2
uset_openPattern(const UChar* pattern, int32_t patternLength,
                 UErrorCode* ec);

U_STABLE USet* U_EXPORT2
uset_openPatternOptions(const UChar* pattern, int32_t patternLength,
                 uint32_t options,
                 UErrorCode* ec);

U_STABLE void U_EXPORT2
uset_close(USet* set);

U_DRAFT void U_EXPORT2
uset_set(USet* set,
         UChar32 start, UChar32 end);

U_DRAFT int32_t U_EXPORT2 
uset_applyPattern(USet *set,
                  const UChar *pattern, int32_t patternLength,
                  uint32_t options,
                  UErrorCode *status);

U_DRAFT void U_EXPORT2
uset_applyIntPropertyValue(USet* set,
                           UProperty prop, int32_t value, UErrorCode* ec);

U_DRAFT void U_EXPORT2
uset_applyPropertyAlias(USet* set,
                        const UChar *prop, int32_t propLength,
                        const UChar *value, int32_t valueLength,
                        UErrorCode* ec);

U_DRAFT UBool U_EXPORT2
uset_resemblesPattern(const UChar *pattern, int32_t patternLength,
                      int32_t pos);

U_STABLE int32_t U_EXPORT2
uset_toPattern(const USet* set,
               UChar* result, int32_t resultCapacity,
               UBool escapeUnprintable,
               UErrorCode* ec);

U_STABLE void U_EXPORT2
uset_add(USet* set, UChar32 c);

U_STABLE void U_EXPORT2
uset_addAll(USet* set, const USet *additionalSet);

U_STABLE void U_EXPORT2
uset_addRange(USet* set, UChar32 start, UChar32 end);

U_STABLE void U_EXPORT2
uset_addString(USet* set, const UChar* str, int32_t strLen);

U_STABLE void U_EXPORT2
uset_remove(USet* set, UChar32 c);

U_STABLE void U_EXPORT2
uset_removeRange(USet* set, UChar32 start, UChar32 end);

U_STABLE void U_EXPORT2
uset_removeString(USet* set, const UChar* str, int32_t strLen);

U_DRAFT void U_EXPORT2
uset_removeAll(USet* set, const USet* removeSet);

U_DRAFT void U_EXPORT2
uset_retain(USet* set, UChar32 start, UChar32 end);

U_DRAFT void U_EXPORT2
uset_retainAll(USet* set, const USet* retain);

U_DRAFT void U_EXPORT2
uset_compact(USet* set);

U_STABLE void U_EXPORT2
uset_complement(USet* set);

U_DRAFT void U_EXPORT2
uset_complementAll(USet* set, const USet* complement);

U_STABLE void U_EXPORT2
uset_clear(USet* set);

U_STABLE UBool U_EXPORT2
uset_isEmpty(const USet* set);

U_STABLE UBool U_EXPORT2
uset_contains(const USet* set, UChar32 c);

U_STABLE UBool U_EXPORT2
uset_containsRange(const USet* set, UChar32 start, UChar32 end);

U_STABLE UBool U_EXPORT2
uset_containsString(const USet* set, const UChar* str, int32_t strLen);

U_DRAFT int32_t U_EXPORT2
uset_indexOf(const USet* set, UChar32 c);

U_DRAFT UChar32 U_EXPORT2
uset_charAt(const USet* set, int32_t index);

U_STABLE int32_t U_EXPORT2
uset_size(const USet* set);

U_STABLE int32_t U_EXPORT2
uset_getItemCount(const USet* set);

U_STABLE int32_t U_EXPORT2
uset_getItem(const USet* set, int32_t itemIndex,
             UChar32* start, UChar32* end,
             UChar* str, int32_t strCapacity,
             UErrorCode* ec);

U_DRAFT UBool U_EXPORT2
uset_containsAll(const USet* set1, const USet* set2);

U_DRAFT UBool U_EXPORT2
uset_containsNone(const USet* set1, const USet* set2);

U_DRAFT UBool U_EXPORT2
uset_containsSome(const USet* set1, const USet* set2);

U_DRAFT UBool U_EXPORT2
uset_equals(const USet* set1, const USet* set2);


U_STABLE int32_t U_EXPORT2
uset_serialize(const USet* set, uint16_t* dest, int32_t destCapacity, UErrorCode* pErrorCode);

U_STABLE UBool U_EXPORT2
uset_getSerializedSet(USerializedSet* fillSet, const uint16_t* src, int32_t srcLength);

U_STABLE void U_EXPORT2
uset_setSerializedToOne(USerializedSet* fillSet, UChar32 c);

U_STABLE UBool U_EXPORT2
uset_serializedContains(const USerializedSet* set, UChar32 c);

U_STABLE int32_t U_EXPORT2
uset_getSerializedRangeCount(const USerializedSet* set);

U_STABLE UBool U_EXPORT2
uset_getSerializedRange(const USerializedSet* set, int32_t rangeIndex,
                        UChar32* pStart, UChar32* pEnd);

#endif
