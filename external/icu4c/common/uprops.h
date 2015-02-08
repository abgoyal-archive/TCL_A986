
#ifndef __UPROPS_H__
#define __UPROPS_H__

#include "unicode/utypes.h"
#include "unicode/uset.h"
#include "uset_imp.h"
#include "udataswp.h"

/* indexes[] entries */
enum {
    UPROPS_PROPS32_INDEX,
    UPROPS_EXCEPTIONS_INDEX,
    UPROPS_EXCEPTIONS_TOP_INDEX,

    UPROPS_ADDITIONAL_TRIE_INDEX,
    UPROPS_ADDITIONAL_VECTORS_INDEX,
    UPROPS_ADDITIONAL_VECTORS_COLUMNS_INDEX,

    UPROPS_RESERVED_INDEX, /* 6 */

    /* maximum values for code values in vector word 0 */
    UPROPS_MAX_VALUES_INDEX=10,
    /* maximum values for code values in vector word 2 */
    UPROPS_MAX_VALUES_2_INDEX,

    UPROPS_INDEX_COUNT=16
};

/* definitions for the main properties words */
enum {
    /* general category shift==0                                0 (5 bits) */
    /* reserved                                                 5 (1 bit) */
    UPROPS_NUMERIC_TYPE_VALUE_SHIFT=6                       /*  6 (10 bits) */
};

#define GET_CATEGORY(props) ((props)&0x1f)
#define CAT_MASK(props) U_MASK(GET_CATEGORY(props))

#define GET_NUMERIC_TYPE_VALUE(props) ((props)>>UPROPS_NUMERIC_TYPE_VALUE_SHIFT)

/* constants for the storage form of numeric types and values */
enum {
    UPROPS_NTV_NONE=0,
    UPROPS_NTV_DECIMAL_START=1,
    UPROPS_NTV_DIGIT_START=11,
    UPROPS_NTV_NUMERIC_START=21,
    UPROPS_NTV_FRACTION_START=0xb0,
    UPROPS_NTV_LARGE_START=0x1e0,
    UPROPS_NTV_RESERVED_START=0x300,

    UPROPS_NTV_MAX_SMALL_INT=UPROPS_NTV_FRACTION_START-UPROPS_NTV_NUMERIC_START-1
};

#define UPROPS_NTV_GET_TYPE(ntv) \
    ((ntv==UPROPS_NTV_NONE) ? U_NT_NONE : \
    (ntv<UPROPS_NTV_DIGIT_START) ?  U_NT_DECIMAL : \
    (ntv<UPROPS_NTV_NUMERIC_START) ? U_NT_DIGIT : \
    U_NT_NUMERIC)

/* number of properties vector words */
#define UPROPS_VECTOR_WORDS     3


/* derived age: one nibble each for major and minor version numbers */
#define UPROPS_AGE_MASK         0xff000000
#define UPROPS_AGE_SHIFT        24

#define UPROPS_EA_MASK          0x000e0000
#define UPROPS_EA_SHIFT         17

#define UPROPS_BLOCK_MASK       0x0001ff00
#define UPROPS_BLOCK_SHIFT      8

#define UPROPS_SCRIPT_MASK      0x000000ff

enum {
    UPROPS_WHITE_SPACE,
    UPROPS_DASH,
    UPROPS_HYPHEN,
    UPROPS_QUOTATION_MARK,
    UPROPS_TERMINAL_PUNCTUATION,
    UPROPS_MATH,
    UPROPS_HEX_DIGIT,
    UPROPS_ASCII_HEX_DIGIT,
    UPROPS_ALPHABETIC,
    UPROPS_IDEOGRAPHIC,
    UPROPS_DIACRITIC,
    UPROPS_EXTENDER,
    UPROPS_NONCHARACTER_CODE_POINT,
    UPROPS_GRAPHEME_EXTEND,
    UPROPS_GRAPHEME_LINK,
    UPROPS_IDS_BINARY_OPERATOR,
    UPROPS_IDS_TRINARY_OPERATOR,
    UPROPS_RADICAL,
    UPROPS_UNIFIED_IDEOGRAPH,
    UPROPS_DEFAULT_IGNORABLE_CODE_POINT,
    UPROPS_DEPRECATED,
    UPROPS_LOGICAL_ORDER_EXCEPTION,
    UPROPS_XID_START,
    UPROPS_XID_CONTINUE,
    UPROPS_ID_START,                            /* ICU 2.6, uprops format version 3.2 */
    UPROPS_ID_CONTINUE,
    UPROPS_GRAPHEME_BASE,
    UPROPS_S_TERM,                              /* new in ICU 3.0 and Unicode 4.0.1 */
    UPROPS_VARIATION_SELECTOR,
    UPROPS_PATTERN_SYNTAX,                      /* new in ICU 3.4 and Unicode 4.1 */
    UPROPS_PATTERN_WHITE_SPACE,
    UPROPS_RESERVED,                            /* reserved & unused */
    UPROPS_BINARY_1_TOP                         /* ==32 - full! */
};

#define UPROPS_LB_MASK          0x03f00000
#define UPROPS_LB_SHIFT         20
#define UPROPS_LB_VWORD         2

#define UPROPS_SB_MASK          0x000f8000
#define UPROPS_SB_SHIFT         15

#define UPROPS_WB_MASK          0x00007c00
#define UPROPS_WB_SHIFT         10

#define UPROPS_GCB_MASK         0x000003e0
#define UPROPS_GCB_SHIFT        5

#define UPROPS_DT_MASK          0x0000001f

U_CFUNC uint32_t
u_getUnicodeProperties(UChar32 c, int32_t column);

U_CFUNC int32_t
uprv_getMaxValues(int32_t column);

U_CFUNC UBool
u_isalnumPOSIX(UChar32 c);

U_CFUNC UBool
u_isgraphPOSIX(UChar32 c);

U_CFUNC UBool
u_isprintPOSIX(UChar32 c);

/** Turn a bit index into a bit flag. @internal */
#define FLAG(n) ((uint32_t)1<<(n))

/** Flags for general categories in the order of UCharCategory. @internal */
#define _Cn     FLAG(U_GENERAL_OTHER_TYPES)
#define _Lu     FLAG(U_UPPERCASE_LETTER)
#define _Ll     FLAG(U_LOWERCASE_LETTER)
#define _Lt     FLAG(U_TITLECASE_LETTER)
#define _Lm     FLAG(U_MODIFIER_LETTER)
/* #define _Lo     FLAG(U_OTHER_LETTER) -- conflicts with MS Visual Studio 9.0 xiosbase */
#define _Mn     FLAG(U_NON_SPACING_MARK)
#define _Me     FLAG(U_ENCLOSING_MARK)
#define _Mc     FLAG(U_COMBINING_SPACING_MARK)
#define _Nd     FLAG(U_DECIMAL_DIGIT_NUMBER)
#define _Nl     FLAG(U_LETTER_NUMBER)
#define _No     FLAG(U_OTHER_NUMBER)
#define _Zs     FLAG(U_SPACE_SEPARATOR)
#define _Zl     FLAG(U_LINE_SEPARATOR)
#define _Zp     FLAG(U_PARAGRAPH_SEPARATOR)
#define _Cc     FLAG(U_CONTROL_CHAR)
#define _Cf     FLAG(U_FORMAT_CHAR)
#define _Co     FLAG(U_PRIVATE_USE_CHAR)
#define _Cs     FLAG(U_SURROGATE)
#define _Pd     FLAG(U_DASH_PUNCTUATION)
#define _Ps     FLAG(U_START_PUNCTUATION)
/* #define _Pe     FLAG(U_END_PUNCTUATION) -- conflicts with MS Visual Studio 9.0 xlocnum */
/* #define _Pc     FLAG(U_CONNECTOR_PUNCTUATION) -- conflicts with MS Visual Studio 9.0 streambuf */
#define _Po     FLAG(U_OTHER_PUNCTUATION)
#define _Sm     FLAG(U_MATH_SYMBOL)
#define _Sc     FLAG(U_CURRENCY_SYMBOL)
#define _Sk     FLAG(U_MODIFIER_SYMBOL)
#define _So     FLAG(U_OTHER_SYMBOL)
#define _Pi     FLAG(U_INITIAL_PUNCTUATION)
/* #define _Pf     FLAG(U_FINAL_PUNCTUATION) -- conflicts with MS Visual Studio 9.0 streambuf */

/** Some code points. @internal */
enum {
    TAB     =0x0009,
    LF      =0x000a,
    FF      =0x000c,
    CR      =0x000d,
    U_A     =0x0041,
    U_F     =0x0046,
    U_Z     =0x005a,
    U_a     =0x0061,
    U_f     =0x0066,
    U_z     =0x007a,
    DEL     =0x007f,
    NL      =0x0085,
    NBSP    =0x00a0,
    CGJ     =0x034f,
    FIGURESP=0x2007,
    HAIRSP  =0x200a,
    ZWNJ    =0x200c,
    ZWJ     =0x200d,
    RLM     =0x200f,
    NNBSP   =0x202f,
    WJ      =0x2060,
    INHSWAP =0x206a,
    NOMDIG  =0x206f,
    U_FW_A  =0xff21,
    U_FW_F  =0xff26,
    U_FW_Z  =0xff3a,
    U_FW_a  =0xff41,
    U_FW_f  =0xff46,
    U_FW_z  =0xff5a,
    ZWNBSP  =0xfeff
};

U_CAPI int32_t U_EXPORT2
uprv_getMaxCharNameLength(void);

U_CAPI void U_EXPORT2
uprv_getCharNameCharacters(const USetAdder *sa);

enum UPropertySource {
    /** No source, not a supported property. */
    UPROPS_SRC_NONE,
    /** From uchar.c/uprops.icu main trie */
    UPROPS_SRC_CHAR,
    /** From uchar.c/uprops.icu properties vectors trie */
    UPROPS_SRC_PROPSVEC,
    /** From unames.c/unames.icu */
    UPROPS_SRC_NAMES,
    /** From unorm.cpp/unorm.icu */
    UPROPS_SRC_NORM,
    /** From ucase.c/ucase.icu */
    UPROPS_SRC_CASE,
    /** From ubidi_props.c/ubidi.icu */
    UPROPS_SRC_BIDI,
    /** From uchar.c/uprops.icu main trie as well as properties vectors trie */
    UPROPS_SRC_CHAR_AND_PROPSVEC,
    /** From ucase.c/ucase.icu as well as unorm.cpp/unorm.icu */
    UPROPS_SRC_CASE_AND_NORM,
    /** From normalizer2impl.cpp/nfc.nrm */
    UPROPS_SRC_NFC,
    /** From normalizer2impl.cpp/nfkc.nrm */
    UPROPS_SRC_NFKC,
    /** From normalizer2impl.cpp/nfkc_cf.nrm */
    UPROPS_SRC_NFKC_CF,
    /** One more than the highest UPropertySource (UPROPS_SRC_) constant. */
    UPROPS_SRC_COUNT
};
typedef enum UPropertySource UPropertySource;

U_CFUNC UPropertySource U_EXPORT2
uprops_getSource(UProperty which);

U_CFUNC void U_EXPORT2
uchar_addPropertyStarts(const USetAdder *sa, UErrorCode *pErrorCode);

U_CFUNC void U_EXPORT2
upropsvec_addPropertyStarts(const USetAdder *sa, UErrorCode *pErrorCode);


U_CAPI int32_t U_EXPORT2
uprops_swap(const UDataSwapper *ds,
            const void *inData, int32_t length, void *outData,
            UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
uchar_swapNames(const UDataSwapper *ds,
                const void *inData, int32_t length, void *outData,
                UErrorCode *pErrorCode);

#ifdef XP_CPLUSPLUS

U_NAMESPACE_BEGIN

class UnicodeSet;

// implemented in uniset_props.cpp
U_CFUNC UnicodeSet *
uniset_getUnicode32Instance(UErrorCode &errorCode);

U_NAMESPACE_END

#endif

#endif
