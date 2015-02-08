
#ifndef BASE_THIRD_PARTY_ICU_ICU_UTF_H_
#define BASE_THIRD_PARTY_ICU_ICU_UTF_H_

#include "base/basictypes.h"

namespace base_icu {

typedef uint32 UChar32;
typedef int8 UBool;

// General ---------------------------------------------------------------------
// from utf.h

#define CBU_SENTINEL (-1)

#define CBU_IS_UNICODE_NONCHAR(c) \
    ((c)>=0xfdd0 && \
     ((uint32)(c)<=0xfdef || ((c)&0xfffe)==0xfffe) && \
     (uint32)(c)<=0x10ffff)

#define CBU_IS_UNICODE_CHAR(c) \
    ((uint32)(c)<0xd800 || \
        ((uint32)(c)>0xdfff && \
         (uint32)(c)<=0x10ffff && \
         !U_IS_UNICODE_NONCHAR(c)))

#define CBU_IS_SURROGATE(c) (((c)&0xfffff800)==0xd800)

#define CBU_IS_SURROGATE_LEAD(c) (((c)&0x400)==0)


// UTF-8 macros ----------------------------------------------------------------
// from utf8.h

extern const uint8 utf8_countTrailBytes[256];

#define CBU8_COUNT_TRAIL_BYTES(leadByte) (base_icu::utf8_countTrailBytes[(uint8)leadByte])

#define CBU8_MASK_LEAD_BYTE(leadByte, countTrailBytes) ((leadByte)&=(1<<(6-(countTrailBytes)))-1)

#define CBU8_IS_SINGLE(c) (((c)&0x80)==0)

#define CBU8_IS_LEAD(c) ((uint8)((c)-0xc0)<0x3e)

#define CBU8_IS_TRAIL(c) (((c)&0xc0)==0x80)

#define CBU8_LENGTH(c) \
    ((uint32)(c)<=0x7f ? 1 : \
        ((uint32)(c)<=0x7ff ? 2 : \
            ((uint32)(c)<=0xd7ff ? 3 : \
                ((uint32)(c)<=0xdfff || (uint32)(c)>0x10ffff ? 0 : \
                    ((uint32)(c)<=0xffff ? 3 : 4)\
                ) \
            ) \
        ) \
    )

#define CBU8_MAX_LENGTH 4

UChar32 utf8_nextCharSafeBody(const uint8 *s, int32 *pi, int32 length, UChar32 c, UBool strict);

#define CBU8_NEXT(s, i, length, c) { \
    (c)=(s)[(i)++]; \
    if(((uint8)(c))>=0x80) { \
        if(CBU8_IS_LEAD(c)) { \
            (c)=base_icu::utf8_nextCharSafeBody((const uint8 *)s, &(i), (int32)(length), c, -1); \
        } else { \
            (c)=CBU_SENTINEL; \
        } \
    } \
}

#define CBU8_APPEND_UNSAFE(s, i, c) { \
    if((uint32)(c)<=0x7f) { \
        (s)[(i)++]=(uint8)(c); \
    } else { \
        if((uint32)(c)<=0x7ff) { \
            (s)[(i)++]=(uint8)(((c)>>6)|0xc0); \
        } else { \
            if((uint32)(c)<=0xffff) { \
                (s)[(i)++]=(uint8)(((c)>>12)|0xe0); \
            } else { \
                (s)[(i)++]=(uint8)(((c)>>18)|0xf0); \
                (s)[(i)++]=(uint8)((((c)>>12)&0x3f)|0x80); \
            } \
            (s)[(i)++]=(uint8)((((c)>>6)&0x3f)|0x80); \
        } \
        (s)[(i)++]=(uint8)(((c)&0x3f)|0x80); \
    } \
}

// UTF-16 macros ---------------------------------------------------------------
// from utf8.h

#define CBU16_IS_SINGLE(c) !U_IS_SURROGATE(c)

#define CBU16_IS_LEAD(c) (((c)&0xfffffc00)==0xd800)

#define CBU16_IS_TRAIL(c) (((c)&0xfffffc00)==0xdc00)

#define CBU16_IS_SURROGATE(c) CBU_IS_SURROGATE(c)

#define CBU16_IS_SURROGATE_LEAD(c) (((c)&0x400)==0)

#define CBU16_SURROGATE_OFFSET ((0xd800<<10UL)+0xdc00-0x10000)

#define CBU16_GET_SUPPLEMENTARY(lead, trail) \
    (((base_icu::UChar32)(lead)<<10UL)+(base_icu::UChar32)(trail)-CBU16_SURROGATE_OFFSET)


#define CBU16_LEAD(supplementary) (UChar)(((supplementary)>>10)+0xd7c0)

#define CBU16_TRAIL(supplementary) (UChar)(((supplementary)&0x3ff)|0xdc00)

#define CBU16_LENGTH(c) ((uint32)(c)<=0xffff ? 1 : 2)

#define CBU16_MAX_LENGTH 2

#define CBU16_APPEND_UNSAFE(s, i, c) { \
    if((uint32)(c)<=0xffff) { \
        (s)[(i)++]=(uint16)(c); \
    } else { \
        (s)[(i)++]=(uint16)(((c)>>10)+0xd7c0); \
        (s)[(i)++]=(uint16)(((c)&0x3ff)|0xdc00); \
    } \
}

}  // namesapce base_icu

#endif  // BASE_THIRD_PARTY_ICU_ICU_UTF_H_
