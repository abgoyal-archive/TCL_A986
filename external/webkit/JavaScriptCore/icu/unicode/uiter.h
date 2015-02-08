

#ifndef __UITER_H__
#define __UITER_H__


#include "unicode/utypes.h"

#ifdef XP_CPLUSPLUS
    U_NAMESPACE_BEGIN

    class CharacterIterator;
    class Replaceable;

    U_NAMESPACE_END
#endif

U_CDECL_BEGIN

struct UCharIterator;
typedef struct UCharIterator UCharIterator; /**< C typedef for struct UCharIterator. @stable ICU 2.1 */

typedef enum UCharIteratorOrigin {
    UITER_START, UITER_CURRENT, UITER_LIMIT, UITER_ZERO, UITER_LENGTH
} UCharIteratorOrigin;

/** Constants for UCharIterator. @stable ICU 2.6 */
enum {
    /**
     * Constant value that may be returned by UCharIteratorMove
     * indicating that the final UTF-16 index is not known, but that the move succeeded.
     * This can occur when moving relative to limit or length, or
     * when moving relative to the current index after a setState()
     * when the current UTF-16 index is not known.
     *
     * It would be very inefficient to have to count from the beginning of the text
     * just to get the current/limit/length index after moving relative to it.
     * The actual index can be determined with getIndex(UITER_CURRENT)
     * which will count the UChars if necessary.
     *
     * @stable ICU 2.6
     */
    UITER_UNKNOWN_INDEX=-2
};


#define UITER_NO_STATE ((uint32_t)0xffffffff)

typedef int32_t U_CALLCONV
UCharIteratorGetIndex(UCharIterator *iter, UCharIteratorOrigin origin);

typedef int32_t U_CALLCONV
UCharIteratorMove(UCharIterator *iter, int32_t delta, UCharIteratorOrigin origin);

typedef UBool U_CALLCONV
UCharIteratorHasNext(UCharIterator *iter);

typedef UBool U_CALLCONV
UCharIteratorHasPrevious(UCharIterator *iter);
 
typedef UChar32 U_CALLCONV
UCharIteratorCurrent(UCharIterator *iter);

typedef UChar32 U_CALLCONV
UCharIteratorNext(UCharIterator *iter);

typedef UChar32 U_CALLCONV
UCharIteratorPrevious(UCharIterator *iter);

typedef int32_t U_CALLCONV
UCharIteratorReserved(UCharIterator *iter, int32_t something);

typedef uint32_t U_CALLCONV
UCharIteratorGetState(const UCharIterator *iter);

typedef void U_CALLCONV
UCharIteratorSetState(UCharIterator *iter, uint32_t state, UErrorCode *pErrorCode);


struct UCharIterator {
    /**
     * (protected) Pointer to string or wrapped object or similar.
     * Not used by caller.
     * @stable ICU 2.1
     */
    const void *context;

    /**
     * (protected) Length of string or similar.
     * Not used by caller.
     * @stable ICU 2.1
     */
    int32_t length;

    /**
     * (protected) Start index or similar.
     * Not used by caller.
     * @stable ICU 2.1
     */
    int32_t start;

    /**
     * (protected) Current index or similar.
     * Not used by caller.
     * @stable ICU 2.1
     */
    int32_t index;

    /**
     * (protected) Limit index or similar.
     * Not used by caller.
     * @stable ICU 2.1
     */
    int32_t limit;

    /**
     * (protected) Used by UTF-8 iterators and possibly others.
     * @stable ICU 2.1
     */
    int32_t reservedField;

    /**
     * (public) Returns the current position or the
     * start or limit index of the iteration range.
     *
     * @see UCharIteratorGetIndex
     * @stable ICU 2.1
     */
    UCharIteratorGetIndex *getIndex;

    /**
     * (public) Moves the current position relative to the start or limit of the
     * iteration range, or relative to the current position itself.
     * The movement is expressed in numbers of code units forward
     * or backward by specifying a positive or negative delta.
     *
     * @see UCharIteratorMove
     * @stable ICU 2.1
     */
    UCharIteratorMove *move;

    /**
     * (public) Check if current() and next() can still
     * return another code unit.
     *
     * @see UCharIteratorHasNext
     * @stable ICU 2.1
     */
    UCharIteratorHasNext *hasNext;

    /**
     * (public) Check if previous() can still return another code unit.
     *
     * @see UCharIteratorHasPrevious
     * @stable ICU 2.1
     */
    UCharIteratorHasPrevious *hasPrevious;

    /**
     * (public) Return the code unit at the current position,
     * or U_SENTINEL if there is none (index is at the limit).
     *
     * @see UCharIteratorCurrent
     * @stable ICU 2.1
     */
    UCharIteratorCurrent *current;

    /**
     * (public) Return the code unit at the current index and increment
     * the index (post-increment, like s[i++]),
     * or return U_SENTINEL if there is none (index is at the limit).
     *
     * @see UCharIteratorNext
     * @stable ICU 2.1
     */
    UCharIteratorNext *next;

    /**
     * (public) Decrement the index and return the code unit from there
     * (pre-decrement, like s[--i]),
     * or return U_SENTINEL if there is none (index is at the start).
     *
     * @see UCharIteratorPrevious
     * @stable ICU 2.1
     */
    UCharIteratorPrevious *previous;

    /**
     * (public) Reserved for future use. Currently NULL.
     *
     * @see UCharIteratorReserved
     * @stable ICU 2.1
     */
    UCharIteratorReserved *reservedFn;

    /**
     * (public) Return the state of the iterator, to be restored later with setState().
     * This function pointer is NULL if the iterator does not implement it.
     *
     * @see UCharIteratorGet
     * @stable ICU 2.6
     */
    UCharIteratorGetState *getState;

    /**
     * (public) Restore the iterator state from the state word from a call
     * to getState().
     * This function pointer is NULL if the iterator does not implement it.
     *
     * @see UCharIteratorSet
     * @stable ICU 2.6
     */
    UCharIteratorSetState *setState;
};

U_STABLE UChar32 U_EXPORT2
uiter_current32(UCharIterator *iter);

U_STABLE UChar32 U_EXPORT2
uiter_next32(UCharIterator *iter);

U_STABLE UChar32 U_EXPORT2
uiter_previous32(UCharIterator *iter);

U_STABLE uint32_t U_EXPORT2
uiter_getState(const UCharIterator *iter);

U_STABLE void U_EXPORT2
uiter_setState(UCharIterator *iter, uint32_t state, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
uiter_setString(UCharIterator *iter, const UChar *s, int32_t length);

U_STABLE void U_EXPORT2
uiter_setUTF16BE(UCharIterator *iter, const char *s, int32_t length);

U_STABLE void U_EXPORT2
uiter_setUTF8(UCharIterator *iter, const char *s, int32_t length);

#ifdef XP_CPLUSPLUS

U_STABLE void U_EXPORT2
uiter_setCharacterIterator(UCharIterator *iter, CharacterIterator *charIter);

U_STABLE void U_EXPORT2
uiter_setReplaceable(UCharIterator *iter, const Replaceable *rep);

#endif

U_CDECL_END

#endif
