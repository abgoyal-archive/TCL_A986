
#ifndef __UNORMIMP_H__
#define __UNORMIMP_H__

#include "unicode/utypes.h"

#if !UCONFIG_NO_NORMALIZATION

#ifdef XP_CPLUSPLUS
#include "unicode/uniset.h"
#endif

#include "unicode/uiter.h"
#include "unicode/unorm.h"
#include "unicode/uset.h"
#include "utrie2.h"
#include "ustr_imp.h"
#include "udataswp.h"


/* norm32 value constants */
enum {
    /* quick check flags 0..3 set mean "no" for their forms */
    _NORM_QC_NFC=0x11,          /* no|maybe */
    _NORM_QC_NFKC=0x22,         /* no|maybe */
    _NORM_QC_NFD=4,             /* no */
    _NORM_QC_NFKD=8,            /* no */

    _NORM_QC_ANY_NO=0xf,

    /* quick check flags 4..5 mean "maybe" for their forms; test flags>=_NORM_QC_MAYBE */
    _NORM_QC_MAYBE=0x10,
    _NORM_QC_ANY_MAYBE=0x30,

    _NORM_QC_MASK=0x3f,

    _NORM_COMBINES_FWD=0x40,
    _NORM_COMBINES_BACK=0x80,
    _NORM_COMBINES_ANY=0xc0,

    _NORM_CC_SHIFT=8,           /* UnicodeData.txt combining class in bits 15..8 */
    _NORM_CC_MASK=0xff00,

    _NORM_EXTRA_SHIFT=16,               /* 16 bits for the index to UChars and other extra data */
    _NORM_EXTRA_INDEX_TOP=0xfc00,       /* start of surrogate specials after shift */

    _NORM_EXTRA_SURROGATE_MASK=0x3ff,
    _NORM_EXTRA_SURROGATE_TOP=0x3f0,    /* hangul etc. */

    _NORM_EXTRA_HANGUL=_NORM_EXTRA_SURROGATE_TOP,
    _NORM_EXTRA_JAMO_L,
    _NORM_EXTRA_JAMO_V,
    _NORM_EXTRA_JAMO_T
};

/* norm32 value constants using >16 bits */
#define _NORM_MIN_SPECIAL       0xfc000000
#define _NORM_SURROGATES_TOP    0xfff00000
#define _NORM_MIN_HANGUL        0xfff00000
#define _NORM_MIN_JAMO_V        0xfff20000
#define _NORM_JAMO_V_TOP        0xfff30000

/* value constants for auxTrie */
enum {
    _NORM_AUX_COMP_EX_SHIFT=10,
    _NORM_AUX_UNSAFE_SHIFT=11,
    _NORM_AUX_NFC_SKIPPABLE_F_SHIFT=12
};

#define _NORM_AUX_MAX_FNC           ((int32_t)1<<_NORM_AUX_COMP_EX_SHIFT)

#define _NORM_AUX_FNC_MASK          (uint32_t)(_NORM_AUX_MAX_FNC-1)
#define _NORM_AUX_COMP_EX_MASK      ((uint32_t)1<<_NORM_AUX_COMP_EX_SHIFT)
#define _NORM_AUX_UNSAFE_MASK       ((uint32_t)1<<_NORM_AUX_UNSAFE_SHIFT)
#define _NORM_AUX_NFC_SKIP_F_MASK   ((uint32_t)1<<_NORM_AUX_NFC_SKIPPABLE_F_SHIFT)

/* canonStartSets[0..31] contains indexes for what is in the array */
enum {
    _NORM_SET_INDEX_CANON_SETS_LENGTH,      /* number of uint16_t in canonical starter sets */
    _NORM_SET_INDEX_CANON_BMP_TABLE_LENGTH, /* number of uint16_t in the BMP search table (contains pairs) */
    _NORM_SET_INDEX_CANON_SUPP_TABLE_LENGTH,/* number of uint16_t in the supplementary search table (contains triplets) */

    /* from formatVersion 2.3: */
    _NORM_SET_INDEX_NX_CJK_COMPAT_OFFSET,   /* uint16_t offset from canonStartSets[0] to the
                                               exclusion set for CJK compatibility characters */
    _NORM_SET_INDEX_NX_UNICODE32_OFFSET,    /* uint16_t offset from canonStartSets[0] to the
                                               exclusion set for Unicode 3.2 characters */
    _NORM_SET_INDEX_NX_RESERVED_OFFSET,     /* uint16_t offset from canonStartSets[0] to the
                                               end of the previous exclusion set */

    _NORM_SET_INDEX_TOP=32                  /* changing this requires a new formatVersion */
};

/* more constants for canonical starter sets */

/* 14 bit indexes to canonical USerializedSets */
#define _NORM_MAX_CANON_SETS            0x4000

/* single-code point BMP sets are encoded directly in the search table except if result=0x4000..0x7fff */
#define _NORM_CANON_SET_BMP_MASK        0xc000
#define _NORM_CANON_SET_BMP_IS_INDEX    0x4000

/* indexes[] value names */
enum {
    _NORM_INDEX_TRIE_SIZE,              /* number of bytes in normalization trie */
    _NORM_INDEX_UCHAR_COUNT,            /* number of UChars in extra data */

    _NORM_INDEX_COMBINE_DATA_COUNT,     /* number of uint16_t words for combining data */
    _NORM_INDEX_COMBINE_FWD_COUNT,      /* number of code points that combine forward */
    _NORM_INDEX_COMBINE_BOTH_COUNT,     /* number of code points that combine forward and backward */
    _NORM_INDEX_COMBINE_BACK_COUNT,     /* number of code points that combine backward */

    _NORM_INDEX_MIN_NFC_NO_MAYBE,       /* first code point with quick check NFC NO/MAYBE */
    _NORM_INDEX_MIN_NFKC_NO_MAYBE,      /* first code point with quick check NFKC NO/MAYBE */
    _NORM_INDEX_MIN_NFD_NO_MAYBE,       /* first code point with quick check NFD NO/MAYBE */
    _NORM_INDEX_MIN_NFKD_NO_MAYBE,      /* first code point with quick check NFKD NO/MAYBE */

    _NORM_INDEX_FCD_TRIE_SIZE,          /* number of bytes in FCD trie */

    _NORM_INDEX_AUX_TRIE_SIZE,          /* number of bytes in the auxiliary trie */
    _NORM_INDEX_CANON_SET_COUNT,        /* number of uint16_t in the array of serialized USet */

    _NORM_INDEX_TOP=32                  /* changing this requires a new formatVersion */
};

enum {
    /* FCD check: everything below this code point is known to have a 0 lead combining class */
    _NORM_MIN_WITH_LEAD_CC=0x300
};

enum {
    /**
     * Bit 7 of the length byte for a decomposition string in extra data is
     * a flag indicating whether the decomposition string is
     * preceded by a 16-bit word with the leading and trailing cc
     * of the decomposition (like for A-umlaut);
     * if not, then both cc's are zero (like for compatibility ideographs).
     */
    _NORM_DECOMP_FLAG_LENGTH_HAS_CC=0x80,
    /**
     * Bits 6..0 of the length byte contain the actual length.
     */
    _NORM_DECOMP_LENGTH_MASK=0x7f
};

/* Constants for options flags for normalization. @draft ICU 2.6 */
enum {
    /** Options bit 0, do not decompose Hangul syllables. @draft ICU 2.6 */
    UNORM_NX_HANGUL=1,
    /** Options bit 1, do not decompose CJK compatibility characters. @draft ICU 2.6 */
    UNORM_NX_CJK_COMPAT=2
};

U_CAPI UBool U_EXPORT2
unorm_haveData(UErrorCode *pErrorCode);

U_CFUNC UBool U_EXPORT2
unorm_isCanonSafeStart(UChar32 c);

U_CAPI UBool U_EXPORT2
unorm_getCanonStartSet(UChar32 c, USerializedSet *fillSet);

U_CAPI void U_EXPORT2
unorm_addPropertyStarts(const USetAdder *sa, UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
unorm_swap(const UDataSwapper *ds,
           const void *inData, int32_t length, void *outData,
           UErrorCode *pErrorCode);


#endif /* #if !UCONFIG_NO_NORMALIZATION */

#endif
