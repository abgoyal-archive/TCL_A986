
#ifndef __UTRIE2_H__
#define __UTRIE2_H__

#include "unicode/utypes.h"
#include "udataswp.h"

U_CDECL_BEGIN

struct UTrie;  /* forward declaration */
#ifndef __UTRIE_H__
typedef struct UTrie UTrie;
#endif


struct UTrie2;
typedef struct UTrie2 UTrie2;

/* Public UTrie2 API functions: read-only access ---------------------------- */

enum UTrie2ValueBits {
    /** 16 bits per UTrie2 data value. */
    UTRIE2_16_VALUE_BITS,
    /** 32 bits per UTrie2 data value. */
    UTRIE2_32_VALUE_BITS,
    /** Number of selectors for the width of UTrie2 data values. */
    UTRIE2_COUNT_VALUE_BITS
};
typedef enum UTrie2ValueBits UTrie2ValueBits;

U_CAPI UTrie2 * U_EXPORT2
utrie2_openFromSerialized(UTrie2ValueBits valueBits,
                          const void *data, int32_t length, int32_t *pActualLength,
                          UErrorCode *pErrorCode);

U_CAPI UTrie2 * U_EXPORT2
utrie2_openDummy(UTrie2ValueBits valueBits,
                 uint32_t initialValue, uint32_t errorValue,
                 UErrorCode *pErrorCode);

U_CAPI uint32_t U_EXPORT2
utrie2_get32(const UTrie2 *trie, UChar32 c);

/* enumeration callback types */

typedef uint32_t U_CALLCONV
UTrie2EnumValue(const void *context, uint32_t value);

typedef UBool U_CALLCONV
UTrie2EnumRange(const void *context, UChar32 start, UChar32 end, uint32_t value);

U_CAPI void U_EXPORT2
utrie2_enum(const UTrie2 *trie,
            UTrie2EnumValue *enumValue, UTrie2EnumRange *enumRange, const void *context);

/* Building a trie ---------------------------------------------------------- */

U_CAPI UTrie2 * U_EXPORT2
utrie2_open(uint32_t initialValue, uint32_t errorValue, UErrorCode *pErrorCode);

U_CAPI UTrie2 * U_EXPORT2
utrie2_clone(const UTrie2 *other, UErrorCode *pErrorCode);

U_CAPI UTrie2 * U_EXPORT2
utrie2_cloneAsThawed(const UTrie2 *other, UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
utrie2_close(UTrie2 *trie);

U_CAPI void U_EXPORT2
utrie2_set32(UTrie2 *trie, UChar32 c, uint32_t value, UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
utrie2_setRange32(UTrie2 *trie,
                  UChar32 start, UChar32 end,
                  uint32_t value, UBool overwrite,
                  UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
utrie2_freeze(UTrie2 *trie, UTrie2ValueBits valueBits, UErrorCode *pErrorCode);

U_CAPI UBool U_EXPORT2
utrie2_isFrozen(const UTrie2 *trie);

U_CAPI int32_t U_EXPORT2
utrie2_serialize(UTrie2 *trie,
                 void *data, int32_t capacity,
                 UErrorCode *pErrorCode);

/* Public UTrie2 API: miscellaneous functions ------------------------------- */

U_CAPI int32_t U_EXPORT2
utrie2_getVersion(const void *data, int32_t length, UBool anyEndianOk);

U_CAPI int32_t U_EXPORT2
utrie2_swap(const UDataSwapper *ds,
            const void *inData, int32_t length, void *outData,
            UErrorCode *pErrorCode);

U_CAPI UTrie2 * U_EXPORT2
utrie2_fromUTrie(const UTrie *trie1, uint32_t errorValue, UErrorCode *pErrorCode);

/* Public UTrie2 API macros ------------------------------------------------- */


#define UTRIE2_GET16(trie, c) _UTRIE2_GET((trie), index, (trie)->indexLength, (c))

#define UTRIE2_GET32(trie, c) _UTRIE2_GET((trie), data32, 0, (c))

#define UTRIE2_U16_NEXT16(trie, src, limit, c, result) _UTRIE2_U16_NEXT(trie, index, src, limit, c, result)

#define UTRIE2_U16_NEXT32(trie, src, limit, c, result) _UTRIE2_U16_NEXT(trie, data32, src, limit, c, result)

#define UTRIE2_U16_PREV16(trie, start, src, c, result) _UTRIE2_U16_PREV(trie, index, start, src, c, result)

#define UTRIE2_U16_PREV32(trie, start, src, c, result) _UTRIE2_U16_PREV(trie, data32, start, src, c, result)

#define UTRIE2_U8_NEXT16(trie, src, limit, result)\
    _UTRIE2_U8_NEXT(trie, data16, index, src, limit, result)

#define UTRIE2_U8_NEXT32(trie, src, limit, result) \
    _UTRIE2_U8_NEXT(trie, data32, data32, src, limit, result)

#define UTRIE2_U8_PREV16(trie, start, src, result) \
    _UTRIE2_U8_PREV(trie, data16, index, start, src, result)

#define UTRIE2_U8_PREV32(trie, start, src, result) \
    _UTRIE2_U8_PREV(trie, data32, data32, start, src, result)

/* Public UTrie2 API: optimized UTF-16 access ------------------------------- */


U_CAPI uint32_t U_EXPORT2
utrie2_get32FromLeadSurrogateCodeUnit(const UTrie2 *trie, UChar32 c);

U_CAPI void U_EXPORT2
utrie2_enumForLeadSurrogate(const UTrie2 *trie, UChar32 lead,
                            UTrie2EnumValue *enumValue, UTrie2EnumRange *enumRange,
                            const void *context);

U_CAPI void U_EXPORT2
utrie2_set32ForLeadSurrogateCodeUnit(UTrie2 *trie,
                                     UChar32 lead, uint32_t value,
                                     UErrorCode *pErrorCode);

#define UTRIE2_GET16_FROM_U16_SINGLE_LEAD(trie, c) _UTRIE2_GET_FROM_U16_SINGLE_LEAD((trie), index, c)

#define UTRIE2_GET32_FROM_U16_SINGLE_LEAD(trie, c) _UTRIE2_GET_FROM_U16_SINGLE_LEAD((trie), data32, c)

#define UTRIE2_GET16_FROM_SUPP(trie, c) _UTRIE2_GET_FROM_SUPP((trie), index, c)

#define UTRIE2_GET32_FROM_SUPP(trie, c) _UTRIE2_GET_FROM_SUPP((trie), data32, c)

U_CDECL_END

/* C++ convenience wrappers ------------------------------------------------- */

#ifdef XP_CPLUSPLUS

#include "mutex.h"

U_NAMESPACE_BEGIN

// Use the Forward/Backward subclasses below.
class UTrie2StringIterator : public UMemory {
public:
    UTrie2StringIterator(const UTrie2 *t, const UChar *p) :
        trie(t), codePointStart(p), codePointLimit(p), codePoint(U_SENTINEL) {}

    const UTrie2 *trie;
    const UChar *codePointStart, *codePointLimit;
    UChar32 codePoint;
};

class BackwardUTrie2StringIterator : public UTrie2StringIterator {
public:
    BackwardUTrie2StringIterator(const UTrie2 *t, const UChar *s, const UChar *p) :
        UTrie2StringIterator(t, p), start(s) {}

    uint16_t previous16();

    const UChar *start;
};

class ForwardUTrie2StringIterator : public UTrie2StringIterator {
public:
    // Iteration limit l can be NULL.
    // In that case, the caller must detect c==0 and stop.
    ForwardUTrie2StringIterator(const UTrie2 *t, const UChar *p, const UChar *l) :
        UTrie2StringIterator(t, p), limit(l) {}

    uint16_t next16();

    const UChar *limit;
};

class UTrie2Singleton {
public:
    UTrie2Singleton(SimpleSingleton &s) : singleton(s) {}
    void deleteInstance() {
        utrie2_close((UTrie2 *)singleton.fInstance);
        singleton.reset();
    }
    UTrie2 *getInstance(InstantiatorFn *instantiator, const void *context,
                        UErrorCode &errorCode);
private:
    SimpleSingleton &singleton;
};

U_NAMESPACE_END

#endif

/* Internal definitions ----------------------------------------------------- */

U_CDECL_BEGIN

/** Build-time trie structure. */
struct UNewTrie2;
typedef struct UNewTrie2 UNewTrie2;

struct UTrie2 {
    /* protected: used by macros and functions for reading values */
    const uint16_t *index;
    const uint16_t *data16;     /* for fast UTF-8 ASCII access, if 16b data */
    const uint32_t *data32;     /* NULL if 16b data is used via index */

    int32_t indexLength, dataLength;
    uint16_t index2NullOffset;  /* 0xffff if there is no dedicated index-2 null block */
    uint16_t dataNullOffset;
    uint32_t initialValue;
    /** Value returned for out-of-range code points and illegal UTF-8. */
    uint32_t errorValue;

    /* Start of the last range which ends at U+10ffff, and its value. */
    UChar32 highStart;
    int32_t highValueIndex;

    /* private: used by builder and unserialization functions */
    void *memory;           /* serialized bytes; NULL if not frozen yet */
    int32_t length;         /* number of serialized bytes at memory; 0 if not frozen yet */
    UBool isMemoryOwned;    /* TRUE if the trie owns the memory */
    UBool padding1;
    int16_t padding2;
    UNewTrie2 *newTrie;     /* builder object; NULL when frozen */
};

enum {
    /** Shift size for getting the index-1 table offset. */
    UTRIE2_SHIFT_1=6+5,

    /** Shift size for getting the index-2 table offset. */
    UTRIE2_SHIFT_2=5,

    /**
     * Difference between the two shift sizes,
     * for getting an index-1 offset from an index-2 offset. 6=11-5
     */
    UTRIE2_SHIFT_1_2=UTRIE2_SHIFT_1-UTRIE2_SHIFT_2,

    /**
     * Number of index-1 entries for the BMP. 32=0x20
     * This part of the index-1 table is omitted from the serialized form.
     */
    UTRIE2_OMITTED_BMP_INDEX_1_LENGTH=0x10000>>UTRIE2_SHIFT_1,

    /** Number of code points per index-1 table entry. 2048=0x800 */
    UTRIE2_CP_PER_INDEX_1_ENTRY=1<<UTRIE2_SHIFT_1,

    /** Number of entries in an index-2 block. 64=0x40 */
    UTRIE2_INDEX_2_BLOCK_LENGTH=1<<UTRIE2_SHIFT_1_2,

    /** Mask for getting the lower bits for the in-index-2-block offset. */
    UTRIE2_INDEX_2_MASK=UTRIE2_INDEX_2_BLOCK_LENGTH-1,

    /** Number of entries in a data block. 32=0x20 */
    UTRIE2_DATA_BLOCK_LENGTH=1<<UTRIE2_SHIFT_2,

    /** Mask for getting the lower bits for the in-data-block offset. */
    UTRIE2_DATA_MASK=UTRIE2_DATA_BLOCK_LENGTH-1,

    /**
     * Shift size for shifting left the index array values.
     * Increases possible data size with 16-bit index values at the cost
     * of compactability.
     * This requires data blocks to be aligned by UTRIE2_DATA_GRANULARITY.
     */
    UTRIE2_INDEX_SHIFT=2,

    /** The alignment size of a data block. Also the granularity for compaction. */
    UTRIE2_DATA_GRANULARITY=1<<UTRIE2_INDEX_SHIFT,

    /* Fixed layout of the first part of the index array. ------------------- */

    /**
     * The BMP part of the index-2 table is fixed and linear and starts at offset 0.
     * Length=2048=0x800=0x10000>>UTRIE2_SHIFT_2.
     */
    UTRIE2_INDEX_2_OFFSET=0,

    /**
     * The part of the index-2 table for U+D800..U+DBFF stores values for
     * lead surrogate code _units_ not code _points_.
     * Values for lead surrogate code _points_ are indexed with this portion of the table.
     * Length=32=0x20=0x400>>UTRIE2_SHIFT_2. (There are 1024=0x400 lead surrogates.)
     */
    UTRIE2_LSCP_INDEX_2_OFFSET=0x10000>>UTRIE2_SHIFT_2,
    UTRIE2_LSCP_INDEX_2_LENGTH=0x400>>UTRIE2_SHIFT_2,

    /** Count the lengths of both BMP pieces. 2080=0x820 */
    UTRIE2_INDEX_2_BMP_LENGTH=UTRIE2_LSCP_INDEX_2_OFFSET+UTRIE2_LSCP_INDEX_2_LENGTH,

    /**
     * The 2-byte UTF-8 version of the index-2 table follows at offset 2080=0x820.
     * Length 32=0x20 for lead bytes C0..DF, regardless of UTRIE2_SHIFT_2.
     */
    UTRIE2_UTF8_2B_INDEX_2_OFFSET=UTRIE2_INDEX_2_BMP_LENGTH,
    UTRIE2_UTF8_2B_INDEX_2_LENGTH=0x800>>6,  /* U+0800 is the first code point after 2-byte UTF-8 */

    /**
     * The index-1 table, only used for supplementary code points, at offset 2112=0x840.
     * Variable length, for code points up to highStart, where the last single-value range starts.
     * Maximum length 512=0x200=0x100000>>UTRIE2_SHIFT_1.
     * (For 0x100000 supplementary code points U+10000..U+10ffff.)
     *
     * The part of the index-2 table for supplementary code points starts
     * after this index-1 table.
     *
     * Both the index-1 table and the following part of the index-2 table
     * are omitted completely if there is only BMP data.
     */
    UTRIE2_INDEX_1_OFFSET=UTRIE2_UTF8_2B_INDEX_2_OFFSET+UTRIE2_UTF8_2B_INDEX_2_LENGTH,
    UTRIE2_MAX_INDEX_1_LENGTH=0x100000>>UTRIE2_SHIFT_1,

    /*
     * Fixed layout of the first part of the data array. -----------------------
     * Starts with 4 blocks (128=0x80 entries) for ASCII.
     */

    /**
     * The illegal-UTF-8 data block follows the ASCII block, at offset 128=0x80.
     * Used with linear access for single bytes 0..0xbf for simple error handling.
     * Length 64=0x40, not UTRIE2_DATA_BLOCK_LENGTH.
     */
    UTRIE2_BAD_UTF8_DATA_OFFSET=0x80,

    /** The start of non-linear-ASCII data blocks, at offset 192=0xc0. */
    UTRIE2_DATA_START_OFFSET=0xc0
};

/* Internal functions and macros -------------------------------------------- */

U_INTERNAL int32_t U_EXPORT2
utrie2_internalU8NextIndex(const UTrie2 *trie, UChar32 c,
                           const uint8_t *src, const uint8_t *limit);

U_INTERNAL int32_t U_EXPORT2
utrie2_internalU8PrevIndex(const UTrie2 *trie, UChar32 c,
                           const uint8_t *start, const uint8_t *src);


/** Internal low-level trie getter. Returns a data index. */
#define _UTRIE2_INDEX_RAW(offset, trieIndex, c) \
    (((int32_t)((trieIndex)[(offset)+((c)>>UTRIE2_SHIFT_2)]) \
    <<UTRIE2_INDEX_SHIFT)+ \
    ((c)&UTRIE2_DATA_MASK))

/** Internal trie getter from a UTF-16 single/lead code unit. Returns the data index. */
#define _UTRIE2_INDEX_FROM_U16_SINGLE_LEAD(trieIndex, c) _UTRIE2_INDEX_RAW(0, trieIndex, c)

/** Internal trie getter from a lead surrogate code point (D800..DBFF). Returns the data index. */
#define _UTRIE2_INDEX_FROM_LSCP(trieIndex, c) \
    _UTRIE2_INDEX_RAW(UTRIE2_LSCP_INDEX_2_OFFSET-(0xd800>>UTRIE2_SHIFT_2), trieIndex, c)

/** Internal trie getter from a BMP code point. Returns the data index. */
#define _UTRIE2_INDEX_FROM_BMP(trieIndex, c) \
    _UTRIE2_INDEX_RAW(U_IS_LEAD(c) ? UTRIE2_LSCP_INDEX_2_OFFSET-(0xd800>>UTRIE2_SHIFT_2) : 0, \
                      trieIndex, c)

/** Internal trie getter from a supplementary code point below highStart. Returns the data index. */
#define _UTRIE2_INDEX_FROM_SUPP(trieIndex, c) \
    (((int32_t)((trieIndex)[ \
        (trieIndex)[(UTRIE2_INDEX_1_OFFSET-UTRIE2_OMITTED_BMP_INDEX_1_LENGTH)+ \
                      ((c)>>UTRIE2_SHIFT_1)]+ \
        (((c)>>UTRIE2_SHIFT_2)&UTRIE2_INDEX_2_MASK)]) \
    <<UTRIE2_INDEX_SHIFT)+ \
    ((c)&UTRIE2_DATA_MASK))

#define _UTRIE2_INDEX_FROM_CP(trie, asciiOffset, c) \
    ((uint32_t)(c)<0xd800 ? \
        _UTRIE2_INDEX_RAW(0, (trie)->index, c) : \
        (uint32_t)(c)<=0xffff ? \
            _UTRIE2_INDEX_RAW( \
                (c)<=0xdbff ? UTRIE2_LSCP_INDEX_2_OFFSET-(0xd800>>UTRIE2_SHIFT_2) : 0, \
                (trie)->index, c) : \
            (uint32_t)(c)>0x10ffff ? \
                (asciiOffset)+UTRIE2_BAD_UTF8_DATA_OFFSET : \
                (c)>=(trie)->highStart ? \
                    (trie)->highValueIndex : \
                    _UTRIE2_INDEX_FROM_SUPP((trie)->index, c))

/** Internal trie getter from a UTF-16 single/lead code unit. Returns the data. */
#define _UTRIE2_GET_FROM_U16_SINGLE_LEAD(trie, data, c) \
    (trie)->data[_UTRIE2_INDEX_FROM_U16_SINGLE_LEAD((trie)->index, c)]

/** Internal trie getter from a supplementary code point. Returns the data. */
#define _UTRIE2_GET_FROM_SUPP(trie, data, c) \
    (trie)->data[(c)>=(trie)->highStart ? (trie)->highValueIndex : \
                 _UTRIE2_INDEX_FROM_SUPP((trie)->index, c)]

#define _UTRIE2_GET(trie, data, asciiOffset, c) \
    (trie)->data[_UTRIE2_INDEX_FROM_CP(trie, asciiOffset, c)]

/** Internal next-post-increment: get the next code point (c) and its data. */
#define _UTRIE2_U16_NEXT(trie, data, src, limit, c, result) { \
    { \
        uint16_t __c2; \
        (c)=*(src)++; \
        if(!U16_IS_LEAD(c)) { \
            (result)=_UTRIE2_GET_FROM_U16_SINGLE_LEAD(trie, data, c); \
        } else if((src)==(limit) || !U16_IS_TRAIL(__c2=*(src))) { \
            (result)=(trie)->data[_UTRIE2_INDEX_FROM_LSCP((trie)->index, c)]; \
        } else { \
            ++(src); \
            (c)=U16_GET_SUPPLEMENTARY((c), __c2); \
            (result)=_UTRIE2_GET_FROM_SUPP((trie), data, (c)); \
        } \
    } \
}

/** Internal pre-decrement-previous: get the previous code point (c) and its data */
#define _UTRIE2_U16_PREV(trie, data, start, src, c, result) { \
    { \
        uint16_t __c2; \
        (c)=*--(src); \
        if(!U16_IS_TRAIL(c) || (src)==(start) || !U16_IS_LEAD(__c2=*((src)-1))) { \
            (result)=(trie)->data[_UTRIE2_INDEX_FROM_BMP((trie)->index, c)]; \
        } else { \
            --(src); \
            (c)=U16_GET_SUPPLEMENTARY(__c2, (c)); \
            (result)=_UTRIE2_GET_FROM_SUPP((trie), data, (c)); \
        } \
    } \
}

/** Internal UTF-8 next-post-increment: get the next code point's data. */
#define _UTRIE2_U8_NEXT(trie, ascii, data, src, limit, result) { \
    uint8_t __lead=(uint8_t)*(src)++; \
    if(__lead<0xc0) { \
        (result)=(trie)->ascii[__lead]; \
    } else { \
        uint8_t __t1, __t2; \
        if( /* handle U+0000..U+07FF inline */ \
            __lead<0xe0 && (src)<(limit) && \
            (__t1=(uint8_t)(*(src)-0x80))<=0x3f \
        ) { \
            ++(src); \
            (result)=(trie)->data[ \
                (trie)->index[(UTRIE2_UTF8_2B_INDEX_2_OFFSET-0xc0)+__lead]+ \
                __t1]; \
        } else if( /* handle U+0000..U+CFFF inline */ \
            __lead<0xed && ((src)+1)<(limit) && \
            (__t1=(uint8_t)(*(src)-0x80))<=0x3f && (__lead>0xe0 || __t1>=0x20) && \
            (__t2=(uint8_t)(*((src)+1)-0x80))<= 0x3f \
        ) { \
            (src)+=2; \
            (result)=(trie)->data[ \
                ((int32_t)((trie)->index[((__lead-0xe0)<<(12-UTRIE2_SHIFT_2))+ \
                                         (__t1<<(6-UTRIE2_SHIFT_2))+(__t2>>UTRIE2_SHIFT_2)]) \
                <<UTRIE2_INDEX_SHIFT)+ \
                (__t2&UTRIE2_DATA_MASK)]; \
        } else { \
            int32_t __index=utrie2_internalU8NextIndex((trie), __lead, (const uint8_t *)(src), \
                                                                       (const uint8_t *)(limit)); \
            (src)+=__index&7; \
            (result)=(trie)->data[__index>>3]; \
        } \
    } \
}

/** Internal UTF-8 pre-decrement-previous: get the previous code point's data. */
#define _UTRIE2_U8_PREV(trie, ascii, data, start, src, result) { \
    uint8_t __b=(uint8_t)*--(src); \
    if(__b<0x80) { \
        (result)=(trie)->ascii[__b]; \
    } else { \
        int32_t __index=utrie2_internalU8PrevIndex((trie), __b, (const uint8_t *)(start), \
                                                                (const uint8_t *)(src)); \
        (src)-=__index&7; \
        (result)=(trie)->data[__index>>3]; \
    } \
}

U_CDECL_END

#if defined (U_HAVE_MSVC_2003_OR_EARLIER)
#pragma optimize("", off)
#endif

#endif
