
#ifndef __UPROPSVEC_H__
#define __UPROPSVEC_H__

#include "unicode/utypes.h"
#include "utrie.h"
#include "utrie2.h"

U_CDECL_BEGIN

struct UPropsVectors;
typedef struct UPropsVectors UPropsVectors;

#define UPVEC_FIRST_SPECIAL_CP 0x110000
#define UPVEC_INITIAL_VALUE_CP 0x110000
#define UPVEC_ERROR_VALUE_CP 0x110001
#define UPVEC_MAX_CP 0x110001

#define UPVEC_START_REAL_VALUES_CP 0x200000

U_CAPI UPropsVectors * U_EXPORT2
upvec_open(int32_t columns, UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
upvec_close(UPropsVectors *pv);

U_CAPI void U_EXPORT2
upvec_setValue(UPropsVectors *pv,
               UChar32 start, UChar32 end,
               int32_t column,
               uint32_t value, uint32_t mask,
               UErrorCode *pErrorCode);

U_CAPI uint32_t U_EXPORT2
upvec_getValue(const UPropsVectors *pv, UChar32 c, int32_t column);

U_CAPI uint32_t * U_EXPORT2
upvec_getRow(const UPropsVectors *pv, int32_t rowIndex,
             UChar32 *pRangeStart, UChar32 *pRangeEnd);

typedef void U_CALLCONV
UPVecCompactHandler(void *context,
                    UChar32 start, UChar32 end,
                    int32_t rowIndex, uint32_t *row, int32_t columns,
                    UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
upvec_compact(UPropsVectors *pv, UPVecCompactHandler *handler, void *context, UErrorCode *pErrorCode);

U_CAPI const uint32_t * U_EXPORT2
upvec_getArray(const UPropsVectors *pv, int32_t *pRows, int32_t *pColumns);

U_CAPI uint32_t * U_EXPORT2
upvec_cloneArray(const UPropsVectors *pv,
                 int32_t *pRows, int32_t *pColumns, UErrorCode *pErrorCode);

U_CAPI UTrie2 * U_EXPORT2
upvec_compactToUTrie2WithRowIndexes(UPropsVectors *pv, UErrorCode *pErrorCode);

struct UPVecToUTrieContext {
    UNewTrie *newTrie;
    int32_t capacity;
    int32_t initialValue;
    UBool latin1Linear;
};
typedef struct UPVecToUTrieContext UPVecToUTrieContext;

/* context=UPVecToUTrieContext, creates the trie and stores the rowIndex values */
U_CAPI void U_CALLCONV
upvec_compactToUTrieHandler(void *context,
                            UChar32 start, UChar32 end,
                            int32_t rowIndex, uint32_t *row, int32_t columns,
                            UErrorCode *pErrorCode);

struct UPVecToUTrie2Context {
    UTrie2 *trie;
    int32_t initialValue;
    int32_t errorValue;
    int32_t maxValue;
};
typedef struct UPVecToUTrie2Context UPVecToUTrie2Context;

/* context=UPVecToUTrie2Context, creates the trie and stores the rowIndex values */
U_CAPI void U_CALLCONV
upvec_compactToUTrie2Handler(void *context,
                             UChar32 start, UChar32 end,
                             int32_t rowIndex, uint32_t *row, int32_t columns,
                             UErrorCode *pErrorCode);

U_CDECL_END

#endif
