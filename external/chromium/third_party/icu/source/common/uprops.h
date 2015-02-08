
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
    UPROPS_NUMERIC_TYPE_SHIFT=5,                            /*  5 (3 bits) */
    UPROPS_NUMERIC_VALUE_SHIFT=8                            /*  8 (8 bits) */
};

#define GET_CATEGORY(props) ((props)&0x1f)
#define CAT_MASK(props) U_MASK(GET_CATEGORY(props))

#define GET_NUMERIC_TYPE(props) (((props)>>UPROPS_NUMERIC_TYPE_SHIFT)&7)
#define GET_NUMERIC_VALUE(props) (((props)>>UPROPS_NUMERIC_VALUE_SHIFT)&0xff)

/* internal numeric pseudo-types for special encodings of numeric values */
enum {
    UPROPS_NT_FRACTION=4, /* ==U_NT_COUNT, must not change unless binary format version changes */
    UPROPS_NT_LARGE,
    UPROPS_NT_COUNT
};

/* encoding of fractional and large numbers */
enum {
    UPROPS_MAX_SMALL_NUMBER=0xff,

    UPROPS_FRACTION_NUM_SHIFT=3,        /* numerator: bits 7..3 */
    UPROPS_FRACTION_DEN_MASK=7,         /* denominator: bits 2..0 */

    UPROPS_FRACTION_MAX_NUM=31,
    UPROPS_FRACTION_DEN_OFFSET=2,       /* denominator values are 2..9 */

    UPROPS_FRACTION_MIN_DEN=UPROPS_FRACTION_DEN_OFFSET,
    UPROPS_FRACTION_MAX_DEN=UPROPS_FRACTION_MIN_DEN+UPROPS_FRACTION_DEN_MASK,

    UPROPS_LARGE_MANT_SHIFT=4,          /* mantissa: bits 7..4 */
    UPROPS_LARGE_EXP_MASK=0xf,          /* exponent: bits 3..0 */
    UPROPS_LARGE_EXP_OFFSET=2,          /* regular exponents 2..17 */
    UPROPS_LARGE_EXP_OFFSET_EXTRA=18,   /* extra large exponents 18..33 */

    UPROPS_LARGE_MIN_EXP=UPROPS_LARGE_EXP_OFFSET,
    UPROPS_LARGE_MAX_EXP=UPROPS_LARGE_MIN_EXP+UPROPS_LARGE_EXP_MASK,
    UPROPS_LARGE_MAX_EXP_EXTRA=UPROPS_LARGE_EXP_OFFSET_EXTRA+UPROPS_LARGE_EXP_MASK
};

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

U_CFUNC UHangulSyllableType
uchar_getHST(UChar32 c);

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
    /** Hangul_Syllable_Type, from uchar.c/uprops.icu */
    UPROPS_SRC_HST,
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

U_CFUNC void U_EXPORT2
uhst_addPropertyStarts(const USetAdder *sa, UErrorCode *pErrorCode);


U_CAPI int32_t U_EXPORT2
uprops_swap(const UDataSwapper *ds,
            const void *inData, int32_t length, void *outData,
            UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
uchar_swapNames(const UDataSwapper *ds,
                const void *inData, int32_t length, void *outData,
                UErrorCode *pErrorCode);

#endif
