
#include "unicode/utypes.h"
#include "unicode/uchar.h"
#include "unicode/uscript.h"
#include "cstring.h"
#include "ucln_cmn.h"
#include "umutex.h"
#include "unormimp.h"
#include "ubidi_props.h"
#include "uprops.h"
#include "ucase.h"

#define LENGTHOF(array) (int32_t)(sizeof(array)/sizeof((array)[0]))

/* cleanup ------------------------------------------------------------------ */

static const UBiDiProps *gBdp=NULL;

static UBool U_CALLCONV uprops_cleanup(void) {
    gBdp=NULL;
    return TRUE;
}

/* bidi/shaping properties API ---------------------------------------------- */

/* get the UBiDiProps singleton, or else its dummy, once and for all */
static const UBiDiProps *
getBiDiProps() {
    /*
     * This lazy intialization with double-checked locking (without mutex protection for
     * the initial check) is transiently unsafe under certain circumstances.
     * Check the readme and use u_init() if necessary.
     */

    /* the initial check is performed by the GET_BIDI_PROPS() macro */
    const UBiDiProps *bdp;
    UErrorCode errorCode=U_ZERO_ERROR;

    bdp=ubidi_getSingleton(&errorCode);
#if !UBIDI_HARDCODE_DATA
    if(U_FAILURE(errorCode)) {
        errorCode=U_ZERO_ERROR;
        bdp=ubidi_getDummy(&errorCode);
        if(U_FAILURE(errorCode)) {
            return NULL;
        }
    }
#endif

    umtx_lock(NULL);
    if(gBdp==NULL) {
        gBdp=bdp;
        ucln_common_registerCleanup(UCLN_COMMON_UPROPS, uprops_cleanup);
    }
    umtx_unlock(NULL);

    return gBdp;
}

/* see comment for GET_CASE_PROPS() */
#define GET_BIDI_PROPS() (gBdp!=NULL ? gBdp : getBiDiProps())

/* general properties API functions ----------------------------------------- */

static const struct {
    int32_t column;
    uint32_t mask;
} binProps[UCHAR_BINARY_LIMIT]={
    /*
     * column and mask values for binary properties from u_getUnicodeProperties().
     * Must be in order of corresponding UProperty,
     * and there must be exacly one entry per binary UProperty.
     *
     * Properties with mask 0 are handled in code.
     * For them, column is the UPropertySource value.
     */
    {  1,               U_MASK(UPROPS_ALPHABETIC) },
    {  1,               U_MASK(UPROPS_ASCII_HEX_DIGIT) },
    { UPROPS_SRC_BIDI,  0 },                                    /* UCHAR_BIDI_CONTROL */
    { UPROPS_SRC_BIDI,  0 },                                    /* UCHAR_BIDI_MIRRORED */
    {  1,               U_MASK(UPROPS_DASH) },
    {  1,               U_MASK(UPROPS_DEFAULT_IGNORABLE_CODE_POINT) },
    {  1,               U_MASK(UPROPS_DEPRECATED) },
    {  1,               U_MASK(UPROPS_DIACRITIC) },
    {  1,               U_MASK(UPROPS_EXTENDER) },
    { UPROPS_SRC_NORM,  0 },                                    /* UCHAR_FULL_COMPOSITION_EXCLUSION */
    {  1,               U_MASK(UPROPS_GRAPHEME_BASE) },
    {  1,               U_MASK(UPROPS_GRAPHEME_EXTEND) },
    {  1,               U_MASK(UPROPS_GRAPHEME_LINK) },
    {  1,               U_MASK(UPROPS_HEX_DIGIT) },
    {  1,               U_MASK(UPROPS_HYPHEN) },
    {  1,               U_MASK(UPROPS_ID_CONTINUE) },
    {  1,               U_MASK(UPROPS_ID_START) },
    {  1,               U_MASK(UPROPS_IDEOGRAPHIC) },
    {  1,               U_MASK(UPROPS_IDS_BINARY_OPERATOR) },
    {  1,               U_MASK(UPROPS_IDS_TRINARY_OPERATOR) },
    { UPROPS_SRC_BIDI,  0 },                                    /* UCHAR_JOIN_CONTROL */
    {  1,               U_MASK(UPROPS_LOGICAL_ORDER_EXCEPTION) },
    { UPROPS_SRC_CASE,  0 },                                    /* UCHAR_LOWERCASE */
    {  1,               U_MASK(UPROPS_MATH) },
    {  1,               U_MASK(UPROPS_NONCHARACTER_CODE_POINT) },
    {  1,               U_MASK(UPROPS_QUOTATION_MARK) },
    {  1,               U_MASK(UPROPS_RADICAL) },
    { UPROPS_SRC_CASE,  0 },                                    /* UCHAR_SOFT_DOTTED */
    {  1,               U_MASK(UPROPS_TERMINAL_PUNCTUATION) },
    {  1,               U_MASK(UPROPS_UNIFIED_IDEOGRAPH) },
    { UPROPS_SRC_CASE,  0 },                                    /* UCHAR_UPPERCASE */
    {  1,               U_MASK(UPROPS_WHITE_SPACE) },
    {  1,               U_MASK(UPROPS_XID_CONTINUE) },
    {  1,               U_MASK(UPROPS_XID_START) },
    { UPROPS_SRC_CASE,  0 },                                    /* UCHAR_CASE_SENSITIVE */
    {  1,               U_MASK(UPROPS_S_TERM) },
    {  1,               U_MASK(UPROPS_VARIATION_SELECTOR) },
    { UPROPS_SRC_NORM,  0 },                                    /* UCHAR_NFD_INERT */
    { UPROPS_SRC_NORM,  0 },                                    /* UCHAR_NFKD_INERT */
    { UPROPS_SRC_NORM,  0 },                                    /* UCHAR_NFC_INERT */
    { UPROPS_SRC_NORM,  0 },                                    /* UCHAR_NFKC_INERT */
    { UPROPS_SRC_NORM,  0 },                                    /* UCHAR_SEGMENT_STARTER */
    {  1,               U_MASK(UPROPS_PATTERN_SYNTAX) },
    {  1,               U_MASK(UPROPS_PATTERN_WHITE_SPACE) },
    { UPROPS_SRC_CHAR_AND_PROPSVEC,  0 },                       /* UCHAR_POSIX_ALNUM */
    { UPROPS_SRC_CHAR,  0 },                                    /* UCHAR_POSIX_BLANK */
    { UPROPS_SRC_CHAR,  0 },                                    /* UCHAR_POSIX_GRAPH */
    { UPROPS_SRC_CHAR,  0 },                                    /* UCHAR_POSIX_PRINT */
    { UPROPS_SRC_CHAR,  0 }                                     /* UCHAR_POSIX_XDIGIT */
};

U_CAPI UBool U_EXPORT2
u_hasBinaryProperty(UChar32 c, UProperty which) {
    /* c is range-checked in the functions that are called from here */
    if(which<UCHAR_BINARY_START || UCHAR_BINARY_LIMIT<=which) {
        /* not a known binary property */
    } else {
        uint32_t mask=binProps[which].mask;
        int32_t column=binProps[which].column;
        if(mask!=0) {
            /* systematic, directly stored properties */
            return (u_getUnicodeProperties(c, column)&mask)!=0;
        } else {
            if(column==UPROPS_SRC_CASE) {
                return ucase_hasBinaryProperty(c, which);
            } else if(column==UPROPS_SRC_NORM) {
#if !UCONFIG_NO_NORMALIZATION
                /* normalization properties from unorm.icu */
                switch(which) {
                case UCHAR_FULL_COMPOSITION_EXCLUSION:
                    return unorm_internalIsFullCompositionExclusion(c);
                case UCHAR_NFD_INERT:
                case UCHAR_NFKD_INERT:
                case UCHAR_NFC_INERT:
                case UCHAR_NFKC_INERT:
                    return unorm_isNFSkippable(c, (UNormalizationMode)(which-UCHAR_NFD_INERT+UNORM_NFD));
                case UCHAR_SEGMENT_STARTER:
                    return unorm_isCanonSafeStart(c);
                default:
                    break;
                }
#endif
            } else if(column==UPROPS_SRC_BIDI) {
                /* bidi/shaping properties */
                const UBiDiProps *bdp=GET_BIDI_PROPS();
                if(bdp!=NULL) {
                    switch(which) {
                    case UCHAR_BIDI_MIRRORED:
                        return ubidi_isMirrored(bdp, c);
                    case UCHAR_BIDI_CONTROL:
                        return ubidi_isBidiControl(bdp, c);
                    case UCHAR_JOIN_CONTROL:
                        return ubidi_isJoinControl(bdp, c);
                    default:
                        break;
                    }
                }
                /* else return FALSE below */
            } else if(column==UPROPS_SRC_CHAR) {
                switch(which) {
                case UCHAR_POSIX_BLANK:
                    return u_isblank(c);
                case UCHAR_POSIX_GRAPH:
                    return u_isgraphPOSIX(c);
                case UCHAR_POSIX_PRINT:
                    return u_isprintPOSIX(c);
                case UCHAR_POSIX_XDIGIT:
                    return u_isxdigit(c);
                default:
                    break;
                }
            } else if(column==UPROPS_SRC_CHAR_AND_PROPSVEC) {
                switch(which) {
                case UCHAR_POSIX_ALNUM:
                    return u_isalnumPOSIX(c);
                default:
                    break;
                }
            }
        }
    }
    return FALSE;
}

U_CAPI int32_t U_EXPORT2
u_getIntPropertyValue(UChar32 c, UProperty which) {
    UErrorCode errorCode;
    int32_t type;

    if(which<UCHAR_BINARY_START) {
        return 0; /* undefined */
    } else if(which<UCHAR_BINARY_LIMIT) {
        return (int32_t)u_hasBinaryProperty(c, which);
    } else if(which<UCHAR_INT_START) {
        return 0; /* undefined */
    } else if(which<UCHAR_INT_LIMIT) {
        switch(which) {
        case UCHAR_BIDI_CLASS:
            return (int32_t)u_charDirection(c);
        case UCHAR_BLOCK:
            return (int32_t)ublock_getCode(c);
        case UCHAR_CANONICAL_COMBINING_CLASS:
#if !UCONFIG_NO_NORMALIZATION
            return u_getCombiningClass(c);
#else
            return 0;
#endif
        case UCHAR_DECOMPOSITION_TYPE:
            return (int32_t)(u_getUnicodeProperties(c, 2)&UPROPS_DT_MASK);
        case UCHAR_EAST_ASIAN_WIDTH:
            return (int32_t)(u_getUnicodeProperties(c, 0)&UPROPS_EA_MASK)>>UPROPS_EA_SHIFT;
        case UCHAR_GENERAL_CATEGORY:
            return (int32_t)u_charType(c);
        case UCHAR_JOINING_GROUP:
            return ubidi_getJoiningGroup(GET_BIDI_PROPS(), c);
        case UCHAR_JOINING_TYPE:
            return ubidi_getJoiningType(GET_BIDI_PROPS(), c);
        case UCHAR_LINE_BREAK:
            return (int32_t)(u_getUnicodeProperties(c, UPROPS_LB_VWORD)&UPROPS_LB_MASK)>>UPROPS_LB_SHIFT;
        case UCHAR_NUMERIC_TYPE:
            type=(int32_t)GET_NUMERIC_TYPE(u_getUnicodeProperties(c, -1));
            if(type>U_NT_NUMERIC) {
                /* keep internal variants of U_NT_NUMERIC from becoming visible */
                type=U_NT_NUMERIC;
            }
            return type;
        case UCHAR_SCRIPT:
            errorCode=U_ZERO_ERROR;
            return (int32_t)uscript_getScript(c, &errorCode);
        case UCHAR_HANGUL_SYLLABLE_TYPE:
            return uchar_getHST(c);
#if !UCONFIG_NO_NORMALIZATION
        case UCHAR_NFD_QUICK_CHECK:
        case UCHAR_NFKD_QUICK_CHECK:
        case UCHAR_NFC_QUICK_CHECK:
        case UCHAR_NFKC_QUICK_CHECK:
            return (int32_t)unorm_getQuickCheck(c, (UNormalizationMode)(which-UCHAR_NFD_QUICK_CHECK+UNORM_NFD));
        case UCHAR_LEAD_CANONICAL_COMBINING_CLASS:
            return unorm_getFCD16FromCodePoint(c)>>8;
        case UCHAR_TRAIL_CANONICAL_COMBINING_CLASS:
            return unorm_getFCD16FromCodePoint(c)&0xff;
#endif
        case UCHAR_GRAPHEME_CLUSTER_BREAK:
            return (int32_t)(u_getUnicodeProperties(c, 2)&UPROPS_GCB_MASK)>>UPROPS_GCB_SHIFT;
        case UCHAR_SENTENCE_BREAK:
            return (int32_t)(u_getUnicodeProperties(c, 2)&UPROPS_SB_MASK)>>UPROPS_SB_SHIFT;
        case UCHAR_WORD_BREAK:
            return (int32_t)(u_getUnicodeProperties(c, 2)&UPROPS_WB_MASK)>>UPROPS_WB_SHIFT;
        default:
            return 0; /* undefined */
        }
    } else if(which==UCHAR_GENERAL_CATEGORY_MASK) {
        return U_MASK(u_charType(c));
    } else {
        return 0; /* undefined */
    }
}

U_CAPI int32_t U_EXPORT2
u_getIntPropertyMinValue(UProperty which) {
    return 0; /* all binary/enum/int properties have a minimum value of 0 */
}

U_CAPI int32_t U_EXPORT2
u_getIntPropertyMaxValue(UProperty which) {
    if(which<UCHAR_BINARY_START) {
        return -1; /* undefined */
    } else if(which<UCHAR_BINARY_LIMIT) {
        return 1; /* maximum TRUE for all binary properties */
    } else if(which<UCHAR_INT_START) {
        return -1; /* undefined */
    } else if(which<UCHAR_INT_LIMIT) {
        switch(which) {
        case UCHAR_BIDI_CLASS:
        case UCHAR_JOINING_GROUP:
        case UCHAR_JOINING_TYPE:
            return ubidi_getMaxValue(GET_BIDI_PROPS(), which);
        case UCHAR_BLOCK:
            return (uprv_getMaxValues(0)&UPROPS_BLOCK_MASK)>>UPROPS_BLOCK_SHIFT;
        case UCHAR_CANONICAL_COMBINING_CLASS:
        case UCHAR_LEAD_CANONICAL_COMBINING_CLASS:
        case UCHAR_TRAIL_CANONICAL_COMBINING_CLASS:
            return 0xff; /* TODO do we need to be more precise, getting the actual maximum? */
        case UCHAR_DECOMPOSITION_TYPE:
            return uprv_getMaxValues(2)&UPROPS_DT_MASK;
        case UCHAR_EAST_ASIAN_WIDTH:
            return (uprv_getMaxValues(0)&UPROPS_EA_MASK)>>UPROPS_EA_SHIFT;
        case UCHAR_GENERAL_CATEGORY:
            return (int32_t)U_CHAR_CATEGORY_COUNT-1;
        case UCHAR_LINE_BREAK:
            return (uprv_getMaxValues(UPROPS_LB_VWORD)&UPROPS_LB_MASK)>>UPROPS_LB_SHIFT;
        case UCHAR_NUMERIC_TYPE:
            return (int32_t)U_NT_COUNT-1;
        case UCHAR_SCRIPT:
            return uprv_getMaxValues(0)&UPROPS_SCRIPT_MASK;
        case UCHAR_HANGUL_SYLLABLE_TYPE:
            return (int32_t)U_HST_COUNT-1;
#if !UCONFIG_NO_NORMALIZATION
        case UCHAR_NFD_QUICK_CHECK:
        case UCHAR_NFKD_QUICK_CHECK:
            return (int32_t)UNORM_YES; /* these are never "maybe", only "no" or "yes" */
        case UCHAR_NFC_QUICK_CHECK:
        case UCHAR_NFKC_QUICK_CHECK:
            return (int32_t)UNORM_MAYBE;
#endif
        case UCHAR_GRAPHEME_CLUSTER_BREAK:
            return (uprv_getMaxValues(2)&UPROPS_GCB_MASK)>>UPROPS_GCB_SHIFT;
        case UCHAR_SENTENCE_BREAK:
            return (uprv_getMaxValues(2)&UPROPS_SB_MASK)>>UPROPS_SB_SHIFT;
        case UCHAR_WORD_BREAK:
            return (uprv_getMaxValues(2)&UPROPS_WB_MASK)>>UPROPS_WB_SHIFT;
        default:
            return -1; /* undefined */
        }
    } else {
        return -1; /* undefined */
    }
}

U_CFUNC UPropertySource U_EXPORT2
uprops_getSource(UProperty which) {
    if(which<UCHAR_BINARY_START) {
        return UPROPS_SRC_NONE; /* undefined */
    } else if(which<UCHAR_BINARY_LIMIT) {
        if(binProps[which].mask!=0) {
            return UPROPS_SRC_PROPSVEC;
        } else {
            return (UPropertySource)binProps[which].column;
        }
    } else if(which<UCHAR_INT_START) {
        return UPROPS_SRC_NONE; /* undefined */
    } else if(which<UCHAR_INT_LIMIT) {
        switch(which) {
        case UCHAR_GENERAL_CATEGORY:
        case UCHAR_NUMERIC_TYPE:
            return UPROPS_SRC_CHAR;

        case UCHAR_HANGUL_SYLLABLE_TYPE:
            return UPROPS_SRC_HST;

        case UCHAR_CANONICAL_COMBINING_CLASS:
        case UCHAR_NFD_QUICK_CHECK:
        case UCHAR_NFKD_QUICK_CHECK:
        case UCHAR_NFC_QUICK_CHECK:
        case UCHAR_NFKC_QUICK_CHECK:
        case UCHAR_LEAD_CANONICAL_COMBINING_CLASS:
        case UCHAR_TRAIL_CANONICAL_COMBINING_CLASS:
            return UPROPS_SRC_NORM;

        case UCHAR_BIDI_CLASS:
        case UCHAR_JOINING_GROUP:
        case UCHAR_JOINING_TYPE:
            return UPROPS_SRC_BIDI;

        default:
            return UPROPS_SRC_PROPSVEC;
        }
    } else if(which<UCHAR_STRING_START) {
        switch(which) {
        case UCHAR_GENERAL_CATEGORY_MASK:
        case UCHAR_NUMERIC_VALUE:
            return UPROPS_SRC_CHAR;

        default:
            return UPROPS_SRC_NONE;
        }
    } else if(which<UCHAR_STRING_LIMIT) {
        switch(which) {
        case UCHAR_AGE:
            return UPROPS_SRC_PROPSVEC;

        case UCHAR_BIDI_MIRRORING_GLYPH:
            return UPROPS_SRC_BIDI;

        case UCHAR_CASE_FOLDING:
        case UCHAR_LOWERCASE_MAPPING:
        case UCHAR_SIMPLE_CASE_FOLDING:
        case UCHAR_SIMPLE_LOWERCASE_MAPPING:
        case UCHAR_SIMPLE_TITLECASE_MAPPING:
        case UCHAR_SIMPLE_UPPERCASE_MAPPING:
        case UCHAR_TITLECASE_MAPPING:
        case UCHAR_UPPERCASE_MAPPING:
            return UPROPS_SRC_CASE;

        case UCHAR_ISO_COMMENT:
        case UCHAR_NAME:
        case UCHAR_UNICODE_1_NAME:
            return UPROPS_SRC_NAMES;

        default:
            return UPROPS_SRC_NONE;
        }
    } else {
        return UPROPS_SRC_NONE; /* undefined */
    }
}


#if 0

U_CAPI void U_EXPORT2
uprv_getInclusions(const USetAdder *sa, UErrorCode *pErrorCode) {
    if(pErrorCode==NULL || U_FAILURE(*pErrorCode)) {
        return;
    }

#if !UCONFIG_NO_NORMALIZATION
    unorm_addPropertyStarts(sa, pErrorCode);
#endif
    uchar_addPropertyStarts(sa, pErrorCode);
    uhst_addPropertyStarts(sa, pErrorCode);
    ucase_addPropertyStarts(ucase_getSingleton(pErrorCode), sa, pErrorCode);
    ubidi_addPropertyStarts(ubidi_getSingleton(pErrorCode), sa, pErrorCode);
}

#endif
