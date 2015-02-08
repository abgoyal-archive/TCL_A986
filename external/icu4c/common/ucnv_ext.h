
#ifndef __UCNV_EXT_H__
#define __UCNV_EXT_H__

#include "unicode/utypes.h"

#if !UCONFIG_NO_CONVERSION

#include "unicode/ucnv.h"
#include "ucnv_cnv.h"

enum {
    UCNV_EXT_INDEXES_LENGTH,            /* 0 */

    UCNV_EXT_TO_U_INDEX,                /* 1 */
    UCNV_EXT_TO_U_LENGTH,
    UCNV_EXT_TO_U_UCHARS_INDEX,
    UCNV_EXT_TO_U_UCHARS_LENGTH,

    UCNV_EXT_FROM_U_UCHARS_INDEX,       /* 5 */
    UCNV_EXT_FROM_U_VALUES_INDEX,
    UCNV_EXT_FROM_U_LENGTH,
    UCNV_EXT_FROM_U_BYTES_INDEX,
    UCNV_EXT_FROM_U_BYTES_LENGTH,

    UCNV_EXT_FROM_U_STAGE_12_INDEX,     /* 10 */
    UCNV_EXT_FROM_U_STAGE_1_LENGTH,
    UCNV_EXT_FROM_U_STAGE_12_LENGTH,
    UCNV_EXT_FROM_U_STAGE_3_INDEX,
    UCNV_EXT_FROM_U_STAGE_3_LENGTH,
    UCNV_EXT_FROM_U_STAGE_3B_INDEX,
    UCNV_EXT_FROM_U_STAGE_3B_LENGTH,

    UCNV_EXT_COUNT_BYTES,               /* 17 */
    UCNV_EXT_COUNT_UCHARS,
    UCNV_EXT_FLAGS,

    UCNV_EXT_RESERVED_INDEX,            /* 20, moves with additional indexes */

    UCNV_EXT_SIZE=31,
    UCNV_EXT_INDEXES_MIN_LENGTH=32
};

/* get the pointer to an extension array from indexes[index] */
#define UCNV_EXT_ARRAY(indexes, index, itemType) \
    ((const itemType *)((const char *)(indexes)+(indexes)[index]))

#define UCNV_GET_MAX_BYTES_PER_UCHAR(indexes) \
    ((indexes)[UCNV_EXT_COUNT_BYTES]&0xff)

/* internal API ------------------------------------------------------------- */

U_CFUNC UBool
ucnv_extInitialMatchToU(UConverter *cnv, const int32_t *cx,
                        int32_t firstLength,
                        const char **src, const char *srcLimit,
                        UChar **target, const UChar *targetLimit,
                        int32_t **offsets, int32_t srcIndex,
                        UBool flush,
                        UErrorCode *pErrorCode);

U_CFUNC UChar32
ucnv_extSimpleMatchToU(const int32_t *cx,
                       const char *source, int32_t length,
                       UBool useFallback);

U_CFUNC void
ucnv_extContinueMatchToU(UConverter *cnv,
                         UConverterToUnicodeArgs *pArgs, int32_t srcIndex,
                         UErrorCode *pErrorCode);


U_CFUNC UBool
ucnv_extInitialMatchFromU(UConverter *cnv, const int32_t *cx,
                          UChar32 cp,
                          const UChar **src, const UChar *srcLimit,
                          char **target, const char *targetLimit,
                          int32_t **offsets, int32_t srcIndex,
                          UBool flush,
                          UErrorCode *pErrorCode);

U_CFUNC int32_t
ucnv_extSimpleMatchFromU(const int32_t *cx,
                         UChar32 cp, uint32_t *pValue,
                         UBool useFallback);

U_CFUNC void
ucnv_extContinueMatchFromU(UConverter *cnv,
                           UConverterFromUnicodeArgs *pArgs, int32_t srcIndex,
                           UErrorCode *pErrorCode);

U_CFUNC void
ucnv_extGetUnicodeSet(const UConverterSharedData *sharedData,
                      const USetAdder *sa,
                      UConverterUnicodeSet which,
                      UConverterSetFilter filter,
                      UErrorCode *pErrorCode);

/* toUnicode helpers -------------------------------------------------------- */

#define UCNV_EXT_TO_U_BYTE_SHIFT 24
#define UCNV_EXT_TO_U_VALUE_MASK 0xffffff
#define UCNV_EXT_TO_U_MIN_CODE_POINT 0x1f0000
#define UCNV_EXT_TO_U_MAX_CODE_POINT 0x2fffff
#define UCNV_EXT_TO_U_ROUNDTRIP_FLAG ((uint32_t)1<<23)
#define UCNV_EXT_TO_U_INDEX_MASK 0x3ffff
#define UCNV_EXT_TO_U_LENGTH_SHIFT 18
#define UCNV_EXT_TO_U_LENGTH_OFFSET 12

/* maximum number of indexed UChars */
#define UCNV_EXT_MAX_UCHARS 19

#define UCNV_EXT_TO_U_MAKE_WORD(byte, value) (((uint32_t)(byte)<<UCNV_EXT_TO_U_BYTE_SHIFT)|(value))

#define UCNV_EXT_TO_U_GET_BYTE(word) ((word)>>UCNV_EXT_TO_U_BYTE_SHIFT)
#define UCNV_EXT_TO_U_GET_VALUE(word) ((word)&UCNV_EXT_TO_U_VALUE_MASK)

#define UCNV_EXT_TO_U_IS_PARTIAL(value) ((value)<UCNV_EXT_TO_U_MIN_CODE_POINT)
#define UCNV_EXT_TO_U_GET_PARTIAL_INDEX(value) (value)

#define UCNV_EXT_TO_U_IS_ROUNDTRIP(value) (((value)&UCNV_EXT_TO_U_ROUNDTRIP_FLAG)!=0)
#define UCNV_EXT_TO_U_MASK_ROUNDTRIP(value) ((value)&~UCNV_EXT_TO_U_ROUNDTRIP_FLAG)

/* use after masking off the roundtrip flag */
#define UCNV_EXT_TO_U_IS_CODE_POINT(value) ((value)<=UCNV_EXT_TO_U_MAX_CODE_POINT)
#define UCNV_EXT_TO_U_GET_CODE_POINT(value) ((value)-UCNV_EXT_TO_U_MIN_CODE_POINT)

#define UCNV_EXT_TO_U_GET_INDEX(value) ((value)&UCNV_EXT_TO_U_INDEX_MASK)
#define UCNV_EXT_TO_U_GET_LENGTH(value) (((value)>>UCNV_EXT_TO_U_LENGTH_SHIFT)-UCNV_EXT_TO_U_LENGTH_OFFSET)

/* fromUnicode helpers ------------------------------------------------------ */

/* most trie constants are shared with ucnvmbcs.h */

/* see similar utrie.h UTRIE_INDEX_SHIFT and UTRIE_DATA_GRANULARITY */
#define UCNV_EXT_STAGE_2_LEFT_SHIFT 2
#define UCNV_EXT_STAGE_3_GRANULARITY 4

/* trie access, returns the stage 3 value=index to stage 3b; s1Index=c>>10 */
#define UCNV_EXT_FROM_U(stage12, stage3, s1Index, c) \
    (stage3)[ ((int32_t)(stage12)[ (stage12)[s1Index] +(((c)>>4)&0x3f) ]<<UCNV_EXT_STAGE_2_LEFT_SHIFT) +((c)&0xf) ]

#define UCNV_EXT_FROM_U_LENGTH_SHIFT 24
#define UCNV_EXT_FROM_U_ROUNDTRIP_FLAG ((uint32_t)1<<31)
#define UCNV_EXT_FROM_U_RESERVED_MASK 0x60000000
#define UCNV_EXT_FROM_U_DATA_MASK 0xffffff

/* special value for "no mapping" to <subchar1> (impossible roundtrip to 0 bytes, value 01) */
#define UCNV_EXT_FROM_U_SUBCHAR1 0x80000001

/* at most 3 bytes in the lower part of the value */
#define UCNV_EXT_FROM_U_MAX_DIRECT_LENGTH 3

/* maximum number of indexed bytes */
#define UCNV_EXT_MAX_BYTES 0x1f

#define UCNV_EXT_FROM_U_IS_PARTIAL(value) (((value)>>UCNV_EXT_FROM_U_LENGTH_SHIFT)==0)
#define UCNV_EXT_FROM_U_GET_PARTIAL_INDEX(value) (value)

#define UCNV_EXT_FROM_U_IS_ROUNDTRIP(value) (((value)&UCNV_EXT_FROM_U_ROUNDTRIP_FLAG)!=0)
#define UCNV_EXT_FROM_U_MASK_ROUNDTRIP(value) ((value)&~UCNV_EXT_FROM_U_ROUNDTRIP_FLAG)

/* get length; masks away all other bits */
#define UCNV_EXT_FROM_U_GET_LENGTH(value) (int32_t)(((value)>>UCNV_EXT_FROM_U_LENGTH_SHIFT)&UCNV_EXT_MAX_BYTES)

/* get bytes or bytes index */
#define UCNV_EXT_FROM_U_GET_DATA(value) ((value)&UCNV_EXT_FROM_U_DATA_MASK)

#endif

#endif
