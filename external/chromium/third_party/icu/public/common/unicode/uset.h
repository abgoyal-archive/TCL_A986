


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
     *
     * The resulting set is a superset of the input for the code points but
     * not for the strings.
     * It performs a case mapping closure of the code points and adds
     * full case folding strings for the code points, and reduces strings of
     * the original set to their full case folding equivalents.
     *
     * This is designed for case-insensitive matches, for example
     * in regular expressions. The full code point case closure allows checking of
     * an input character directly against the closure set.
     * Strings are matched by comparing the case-folded form from the closure
     * set with an incremental case folding of the string in question.
     *
     * The closure set will also contain single code points if the original
     * set contained case-equivalent strings (like U+00DF for "ss" or "Ss" etc.).
     * This is not necessary (that is, redundant) for the above matching method
     * but results in the same closure sets regardless of whether the original
     * set contained the code point or a string.
     *
     * @stable ICU 2.4
     */
    USET_CASE_INSENSITIVE = 2,  

    /**
     * Enable case insensitive matching.  E.g., "[ab]" with this flag
     * will match 'a', 'A', 'b', and 'B'.  "[^ab]" with this flag will
     * match all except 'a', 'A', 'b', and 'B'. This adds the lower-,
     * title-, and uppercase mappings as well as the case folding
     * of each existing element in the set.
     * @stable ICU 3.2
     */
    USET_ADD_CASE_MAPPINGS = 4,
    
    /**
     * Enough for any single-code point set
     * @internal
     */
    USET_SERIALIZED_STATIC_ARRAY_CAPACITY=8
};

typedef enum USetSpanCondition {
    /**
     * Continue a span() while there is no set element at the current position.
     * Stops before the first set element (character or string).
     * (For code points only, this is like while contains(current)==FALSE).
     *
     * When span() returns, the substring between where it started and the position
     * it returned consists only of characters that are not in the set,
     * and none of its strings overlap with the span.
     *
     * @stable ICU 3.8
     */
    USET_SPAN_NOT_CONTAINED = 0,
    /**
     * Continue a span() while there is a set element at the current position.
     * (For characters only, this is like while contains(current)==TRUE).
     *
     * When span() returns, the substring between where it started and the position
     * it returned consists only of set elements (characters or strings) that are in the set.
     *
     * If a set contains strings, then the span will be the longest substring
     * matching any of the possible concatenations of set elements (characters or strings).
     * (There must be a single, non-overlapping concatenation of characters or strings.)
     * This is equivalent to a POSIX regular expression for (OR of each set element)*.
     *
     * @stable ICU 3.8
     */
    USET_SPAN_CONTAINED = 1,
    /**
     * Continue a span() while there is a set element at the current position.
     * (For characters only, this is like while contains(current)==TRUE).
     *
     * When span() returns, the substring between where it started and the position
     * it returned consists only of set elements (characters or strings) that are in the set.
     *
     * If a set only contains single characters, then this is the same
     * as USET_SPAN_CONTAINED.
     *
     * If a set contains strings, then the span will be the longest substring
     * with a match at each position with the longest single set element (character or string).
     *
     * Use this span condition together with other longest-match algorithms,
     * such as ICU converters (ucnv_getUnicodeSet()).
     *
     * @stable ICU 3.8
     */
    USET_SPAN_SIMPLE = 2,
    /**
     * One more than the last span condition.
     * @stable ICU 3.8
     */
    USET_SPAN_CONDITION_COUNT
} USetSpanCondition;

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


U_DRAFT USet* U_EXPORT2
uset_openEmpty();

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

U_DRAFT USet * U_EXPORT2
uset_clone(const USet *set);

U_DRAFT UBool U_EXPORT2
uset_isFrozen(const USet *set);

U_DRAFT void U_EXPORT2
uset_freeze(USet *set);

U_DRAFT USet * U_EXPORT2
uset_cloneAsThawed(const USet *set);

U_STABLE void U_EXPORT2
uset_set(USet* set,
         UChar32 start, UChar32 end);

U_STABLE int32_t U_EXPORT2 
uset_applyPattern(USet *set,
                  const UChar *pattern, int32_t patternLength,
                  uint32_t options,
                  UErrorCode *status);

U_STABLE void U_EXPORT2
uset_applyIntPropertyValue(USet* set,
                           UProperty prop, int32_t value, UErrorCode* ec);

U_STABLE void U_EXPORT2
uset_applyPropertyAlias(USet* set,
                        const UChar *prop, int32_t propLength,
                        const UChar *value, int32_t valueLength,
                        UErrorCode* ec);

U_STABLE UBool U_EXPORT2
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
uset_addAllCodePoints(USet* set, const UChar *str, int32_t strLen);

U_STABLE void U_EXPORT2
uset_remove(USet* set, UChar32 c);

U_STABLE void U_EXPORT2
uset_removeRange(USet* set, UChar32 start, UChar32 end);

U_STABLE void U_EXPORT2
uset_removeString(USet* set, const UChar* str, int32_t strLen);

U_STABLE void U_EXPORT2
uset_removeAll(USet* set, const USet* removeSet);

U_STABLE void U_EXPORT2
uset_retain(USet* set, UChar32 start, UChar32 end);

U_STABLE void U_EXPORT2
uset_retainAll(USet* set, const USet* retain);

U_STABLE void U_EXPORT2
uset_compact(USet* set);

U_STABLE void U_EXPORT2
uset_complement(USet* set);

U_STABLE void U_EXPORT2
uset_complementAll(USet* set, const USet* complement);

U_STABLE void U_EXPORT2
uset_clear(USet* set);

U_DRAFT void U_EXPORT2
uset_closeOver(USet* set, int32_t attributes);

U_DRAFT void U_EXPORT2
uset_removeAllStrings(USet* set);

U_STABLE UBool U_EXPORT2
uset_isEmpty(const USet* set);

U_STABLE UBool U_EXPORT2
uset_contains(const USet* set, UChar32 c);

U_STABLE UBool U_EXPORT2
uset_containsRange(const USet* set, UChar32 start, UChar32 end);

U_STABLE UBool U_EXPORT2
uset_containsString(const USet* set, const UChar* str, int32_t strLen);

U_STABLE int32_t U_EXPORT2
uset_indexOf(const USet* set, UChar32 c);

U_STABLE UChar32 U_EXPORT2
uset_charAt(const USet* set, int32_t charIndex);

U_STABLE int32_t U_EXPORT2
uset_size(const USet* set);

U_STABLE int32_t U_EXPORT2
uset_getItemCount(const USet* set);

U_STABLE int32_t U_EXPORT2
uset_getItem(const USet* set, int32_t itemIndex,
             UChar32* start, UChar32* end,
             UChar* str, int32_t strCapacity,
             UErrorCode* ec);

U_STABLE UBool U_EXPORT2
uset_containsAll(const USet* set1, const USet* set2);

U_STABLE UBool U_EXPORT2
uset_containsAllCodePoints(const USet* set, const UChar *str, int32_t strLen);

U_STABLE UBool U_EXPORT2
uset_containsNone(const USet* set1, const USet* set2);

U_STABLE UBool U_EXPORT2
uset_containsSome(const USet* set1, const USet* set2);

U_DRAFT int32_t U_EXPORT2
uset_span(const USet *set, const UChar *s, int32_t length, USetSpanCondition spanCondition);

U_DRAFT int32_t U_EXPORT2
uset_spanBack(const USet *set, const UChar *s, int32_t length, USetSpanCondition spanCondition);

U_DRAFT int32_t U_EXPORT2
uset_spanUTF8(const USet *set, const char *s, int32_t length, USetSpanCondition spanCondition);

U_DRAFT int32_t U_EXPORT2
uset_spanBackUTF8(const USet *set, const char *s, int32_t length, USetSpanCondition spanCondition);

U_STABLE UBool U_EXPORT2
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
