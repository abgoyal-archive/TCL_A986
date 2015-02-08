

#ifndef UCHAR_H
#define UCHAR_H

#include "unicode/utypes.h"

U_CDECL_BEGIN

/*==========================================================================*/
/* Unicode version number                                                   */
/*==========================================================================*/
#define U_UNICODE_VERSION "4.0.1"



/** The lowest Unicode code point value. Code points are non-negative. @stable ICU 2.0 */
#define UCHAR_MIN_VALUE 0

#define UCHAR_MAX_VALUE 0x10ffff

#define U_MASK(x) ((uint32_t)1<<(x))


typedef enum UProperty {
    /*  See note !!.  Comments of the form "Binary property Dash",
        "Enumerated property Script", "Double property Numeric_Value",
        and "String property Age" are read by genpname. */

    /*  Note: Place UCHAR_ALPHABETIC before UCHAR_BINARY_START so that
    debuggers display UCHAR_ALPHABETIC as the symbolic name for 0,
    rather than UCHAR_BINARY_START.  Likewise for other *_START
    identifiers. */

    /** Binary property Alphabetic. Same as u_isUAlphabetic, different from u_isalpha.
        Lu+Ll+Lt+Lm+Lo+Nl+Other_Alphabetic @stable ICU 2.1 */
    UCHAR_ALPHABETIC=0,
    /** First constant for binary Unicode properties. @stable ICU 2.1 */
    UCHAR_BINARY_START=UCHAR_ALPHABETIC,
    /** Binary property ASCII_Hex_Digit. 0-9 A-F a-f @stable ICU 2.1 */
    UCHAR_ASCII_HEX_DIGIT,
    /** Binary property Bidi_Control.
        Format controls which have specific functions
        in the Bidi Algorithm. @stable ICU 2.1 */
    UCHAR_BIDI_CONTROL,
    /** Binary property Bidi_Mirrored.
        Characters that may change display in RTL text.
        Same as u_isMirrored.
        See Bidi Algorithm, UTR 9. @stable ICU 2.1 */
    UCHAR_BIDI_MIRRORED,
    /** Binary property Dash. Variations of dashes. @stable ICU 2.1 */
    UCHAR_DASH,
    /** Binary property Default_Ignorable_Code_Point (new in Unicode 3.2).
        Ignorable in most processing.
        <2060..206F, FFF0..FFFB, E0000..E0FFF>+Other_Default_Ignorable_Code_Point+(Cf+Cc+Cs-White_Space) @stable ICU 2.1 */
    UCHAR_DEFAULT_IGNORABLE_CODE_POINT,
    /** Binary property Deprecated (new in Unicode 3.2).
        The usage of deprecated characters is strongly discouraged. @stable ICU 2.1 */
    UCHAR_DEPRECATED,
    /** Binary property Diacritic. Characters that linguistically modify
        the meaning of another character to which they apply. @stable ICU 2.1 */
    UCHAR_DIACRITIC,
    /** Binary property Extender.
        Extend the value or shape of a preceding alphabetic character,
        e.g., length and iteration marks. @stable ICU 2.1 */
    UCHAR_EXTENDER,
    /** Binary property Full_Composition_Exclusion.
        CompositionExclusions.txt+Singleton Decompositions+
        Non-Starter Decompositions. @stable ICU 2.1 */
    UCHAR_FULL_COMPOSITION_EXCLUSION,
    /** Binary property Grapheme_Base (new in Unicode 3.2).
        For programmatic determination of grapheme cluster boundaries.
        [0..10FFFF]-Cc-Cf-Cs-Co-Cn-Zl-Zp-Grapheme_Link-Grapheme_Extend-CGJ @stable ICU 2.1 */
    UCHAR_GRAPHEME_BASE,
    /** Binary property Grapheme_Extend (new in Unicode 3.2).
        For programmatic determination of grapheme cluster boundaries.
        Me+Mn+Mc+Other_Grapheme_Extend-Grapheme_Link-CGJ @stable ICU 2.1 */
    UCHAR_GRAPHEME_EXTEND,
    /** Binary property Grapheme_Link (new in Unicode 3.2).
        For programmatic determination of grapheme cluster boundaries. @stable ICU 2.1 */
    UCHAR_GRAPHEME_LINK,
    /** Binary property Hex_Digit.
        Characters commonly used for hexadecimal numbers. @stable ICU 2.1 */
    UCHAR_HEX_DIGIT,
    /** Binary property Hyphen. Dashes used to mark connections
        between pieces of words, plus the Katakana middle dot. @stable ICU 2.1 */
    UCHAR_HYPHEN,
    /** Binary property ID_Continue.
        Characters that can continue an identifier.
        DerivedCoreProperties.txt also says "NOTE: Cf characters should be filtered out."
        ID_Start+Mn+Mc+Nd+Pc @stable ICU 2.1 */
    UCHAR_ID_CONTINUE,
    /** Binary property ID_Start.
        Characters that can start an identifier.
        Lu+Ll+Lt+Lm+Lo+Nl @stable ICU 2.1 */
    UCHAR_ID_START,
    /** Binary property Ideographic.
        CJKV ideographs. @stable ICU 2.1 */
    UCHAR_IDEOGRAPHIC,
    /** Binary property IDS_Binary_Operator (new in Unicode 3.2).
        For programmatic determination of
        Ideographic Description Sequences. @stable ICU 2.1 */
    UCHAR_IDS_BINARY_OPERATOR,
    /** Binary property IDS_Trinary_Operator (new in Unicode 3.2).
        For programmatic determination of
        Ideographic Description Sequences. @stable ICU 2.1 */
    UCHAR_IDS_TRINARY_OPERATOR,
    /** Binary property Join_Control.
        Format controls for cursive joining and ligation. @stable ICU 2.1 */
    UCHAR_JOIN_CONTROL,
    /** Binary property Logical_Order_Exception (new in Unicode 3.2).
        Characters that do not use logical order and
        require special handling in most processing. @stable ICU 2.1 */
    UCHAR_LOGICAL_ORDER_EXCEPTION,
    /** Binary property Lowercase. Same as u_isULowercase, different from u_islower.
        Ll+Other_Lowercase @stable ICU 2.1 */
    UCHAR_LOWERCASE,
    /** Binary property Math. Sm+Other_Math @stable ICU 2.1 */
    UCHAR_MATH,
    /** Binary property Noncharacter_Code_Point.
        Code points that are explicitly defined as illegal
        for the encoding of characters. @stable ICU 2.1 */
    UCHAR_NONCHARACTER_CODE_POINT,
    /** Binary property Quotation_Mark. @stable ICU 2.1 */
    UCHAR_QUOTATION_MARK,
    /** Binary property Radical (new in Unicode 3.2).
        For programmatic determination of
        Ideographic Description Sequences. @stable ICU 2.1 */
    UCHAR_RADICAL,
    /** Binary property Soft_Dotted (new in Unicode 3.2).
        Characters with a "soft dot", like i or j.
        An accent placed on these characters causes
        the dot to disappear. @stable ICU 2.1 */
    UCHAR_SOFT_DOTTED,
    /** Binary property Terminal_Punctuation.
        Punctuation characters that generally mark
        the end of textual units. @stable ICU 2.1 */
    UCHAR_TERMINAL_PUNCTUATION,
    /** Binary property Unified_Ideograph (new in Unicode 3.2).
        For programmatic determination of
        Ideographic Description Sequences. @stable ICU 2.1 */
    UCHAR_UNIFIED_IDEOGRAPH,
    /** Binary property Uppercase. Same as u_isUUppercase, different from u_isupper.
        Lu+Other_Uppercase @stable ICU 2.1 */
    UCHAR_UPPERCASE,
    /** Binary property White_Space.
        Same as u_isUWhiteSpace, different from u_isspace and u_isWhitespace.
        Space characters+TAB+CR+LF-ZWSP-ZWNBSP @stable ICU 2.1 */
    UCHAR_WHITE_SPACE,
    /** Binary property XID_Continue.
        ID_Continue modified to allow closure under
        normalization forms NFKC and NFKD. @stable ICU 2.1 */
    UCHAR_XID_CONTINUE,
    /** Binary property XID_Start. ID_Start modified to allow
        closure under normalization forms NFKC and NFKD. @stable ICU 2.1 */
    UCHAR_XID_START,
    /** Binary property Case_Sensitive. Either the source of a case
        mapping or _in_ the target of a case mapping. Not the same as
        the general category Cased_Letter. @stable ICU 2.6 */
    UCHAR_CASE_SENSITIVE,
    /** Binary property STerm (new in Unicode 4.0.1).
        Sentence Terminal. Used in UAX #29: Text Boundaries
        (http://www.unicode.org/reports/tr29/)
        @draft ICU 3.0 */
    UCHAR_S_TERM,
    /** Binary property Variation_Selector (new in Unicode 4.0.1).
        Indicates all those characters that qualify as Variation Selectors.
        For details on the behavior of these characters,
        see StandardizedVariants.html and 15.6 Variation Selectors.
        @draft ICU 3.0 */
    UCHAR_VARIATION_SELECTOR,
    /** Binary property NFD_Inert.
        ICU-specific property for characters that are inert under NFD,
        i.e., they do not interact with adjacent characters.
        Used for example in normalizing transforms in incremental mode
        to find the boundary of safely normalizable text despite possible
        text additions.

        There is one such property per normalization form.
        These properties are computed as follows - an inert character is:
        a) unassigned, or ALL of the following:
        b) of combining class 0.
        c) not decomposed by this normalization form.
        AND if NFC or NFKC,
        d) can never compose with a previous character.
        e) can never compose with a following character.
        f) can never change if another character is added.
           Example: a-breve might satisfy all but f, but if you
           add an ogonek it changes to a-ogonek + breve

        See also com.ibm.text.UCD.NFSkippable in the ICU4J repository,
        and icu/source/common/unormimp.h .
        @draft ICU 3.0 */
    UCHAR_NFD_INERT,
    /** Binary property NFKD_Inert.
        ICU-specific property for characters that are inert under NFKD,
        i.e., they do not interact with adjacent characters.
        Used for example in normalizing transforms in incremental mode
        to find the boundary of safely normalizable text despite possible
        text additions.
        @see UCHAR_NFD_INERT
        @draft ICU 3.0 */
    UCHAR_NFKD_INERT,
    /** Binary property NFC_Inert.
        ICU-specific property for characters that are inert under NFC,
        i.e., they do not interact with adjacent characters.
        Used for example in normalizing transforms in incremental mode
        to find the boundary of safely normalizable text despite possible
        text additions.
        @see UCHAR_NFD_INERT
        @draft ICU 3.0 */
    UCHAR_NFC_INERT,
    /** Binary property NFKC_Inert.
        ICU-specific property for characters that are inert under NFKC,
        i.e., they do not interact with adjacent characters.
        Used for example in normalizing transforms in incremental mode
        to find the boundary of safely normalizable text despite possible
        text additions.
        @see UCHAR_NFD_INERT
        @draft ICU 3.0 */
    UCHAR_NFKC_INERT,
    /** Binary Property Segment_Starter.
        ICU-specific property for characters that are starters in terms of
        Unicode normalization and combining character sequences.
        They have ccc=0 and do not occur in non-initial position of the
        canonical decomposition of any character
        (like " in NFD(a-umlaut) and a Jamo T in an NFD(Hangul LVT)).
        ICU uses this property for segmenting a string for generating a set of
        canonically equivalent strings, e.g. for canonical closure while
        processing collation tailoring rules.
        @draft ICU 3.0 */
    UCHAR_SEGMENT_STARTER,
    /** One more than the last constant for binary Unicode properties. @stable ICU 2.1 */
    UCHAR_BINARY_LIMIT,

    /** Enumerated property Bidi_Class.
        Same as u_charDirection, returns UCharDirection values. @stable ICU 2.2 */
    UCHAR_BIDI_CLASS=0x1000,
    /** First constant for enumerated/integer Unicode properties. @stable ICU 2.2 */
    UCHAR_INT_START=UCHAR_BIDI_CLASS,
    /** Enumerated property Block.
        Same as ublock_getCode, returns UBlockCode values. @stable ICU 2.2 */
    UCHAR_BLOCK,
    /** Enumerated property Canonical_Combining_Class.
        Same as u_getCombiningClass, returns 8-bit numeric values. @stable ICU 2.2 */
    UCHAR_CANONICAL_COMBINING_CLASS,
    /** Enumerated property Decomposition_Type.
        Returns UDecompositionType values. @stable ICU 2.2 */
    UCHAR_DECOMPOSITION_TYPE,
    /** Enumerated property East_Asian_Width.
        See http://www.unicode.org/reports/tr11/
        Returns UEastAsianWidth values. @stable ICU 2.2 */
    UCHAR_EAST_ASIAN_WIDTH,
    /** Enumerated property General_Category.
        Same as u_charType, returns UCharCategory values. @stable ICU 2.2 */
    UCHAR_GENERAL_CATEGORY,
    /** Enumerated property Joining_Group.
        Returns UJoiningGroup values. @stable ICU 2.2 */
    UCHAR_JOINING_GROUP,
    /** Enumerated property Joining_Type.
        Returns UJoiningType values. @stable ICU 2.2 */
    UCHAR_JOINING_TYPE,
    /** Enumerated property Line_Break.
        Returns ULineBreak values. @stable ICU 2.2 */
    UCHAR_LINE_BREAK,
    /** Enumerated property Numeric_Type.
        Returns UNumericType values. @stable ICU 2.2 */
    UCHAR_NUMERIC_TYPE,
    /** Enumerated property Script.
        Same as uscript_getScript, returns UScriptCode values. @stable ICU 2.2 */
    UCHAR_SCRIPT,
    /** Enumerated property Hangul_Syllable_Type, new in Unicode 4.
        Returns UHangulSyllableType values. @stable ICU 2.6 */
    UCHAR_HANGUL_SYLLABLE_TYPE,
    /** Enumerated property NFD_Quick_Check.
        Returns UNormalizationCheckResult values. @draft ICU 3.0 */
    UCHAR_NFD_QUICK_CHECK,
    /** Enumerated property NFKD_Quick_Check.
        Returns UNormalizationCheckResult values. @draft ICU 3.0 */
    UCHAR_NFKD_QUICK_CHECK,
    /** Enumerated property NFC_Quick_Check.
        Returns UNormalizationCheckResult values. @draft ICU 3.0 */
    UCHAR_NFC_QUICK_CHECK,
    /** Enumerated property NFKC_Quick_Check.
        Returns UNormalizationCheckResult values. @draft ICU 3.0 */
    UCHAR_NFKC_QUICK_CHECK,
    /** Enumerated property Lead_Canonical_Combining_Class.
        ICU-specific property for the ccc of the first code point
        of the decomposition, or lccc(c)=ccc(NFD(c)[0]).
        Useful for checking for canonically ordered text;
        see UNORM_FCD and http://www.unicode.org/notes/tn5/#FCD .
        Returns 8-bit numeric values like UCHAR_CANONICAL_COMBINING_CLASS. @draft ICU 3.0 */
    UCHAR_LEAD_CANONICAL_COMBINING_CLASS,
    /** Enumerated property Trail_Canonical_Combining_Class.
        ICU-specific property for the ccc of the last code point
        of the decomposition, or tccc(c)=ccc(NFD(c)[last]).
        Useful for checking for canonically ordered text;
        see UNORM_FCD and http://www.unicode.org/notes/tn5/#FCD .
        Returns 8-bit numeric values like UCHAR_CANONICAL_COMBINING_CLASS. @draft ICU 3.0 */
    UCHAR_TRAIL_CANONICAL_COMBINING_CLASS,
    /** One more than the last constant for enumerated/integer Unicode properties. @stable ICU 2.2 */
    UCHAR_INT_LIMIT,

    /** Bitmask property General_Category_Mask.
        This is the General_Category property returned as a bit mask.
        When used in u_getIntPropertyValue(c), same as U_MASK(u_charType(c)),
        returns bit masks for UCharCategory values where exactly one bit is set.
        When used with u_getPropertyValueName() and u_getPropertyValueEnum(),
        a multi-bit mask is used for sets of categories like "Letters".
        Mask values should be cast to uint32_t.
        @stable ICU 2.4 */
    UCHAR_GENERAL_CATEGORY_MASK=0x2000,
    /** First constant for bit-mask Unicode properties. @stable ICU 2.4 */
    UCHAR_MASK_START=UCHAR_GENERAL_CATEGORY_MASK,
    /** One more than the last constant for bit-mask Unicode properties. @stable ICU 2.4 */
    UCHAR_MASK_LIMIT,

    /** Double property Numeric_Value.
        Corresponds to u_getNumericValue. @stable ICU 2.4 */
    UCHAR_NUMERIC_VALUE=0x3000,
    /** First constant for double Unicode properties. @stable ICU 2.4 */
    UCHAR_DOUBLE_START=UCHAR_NUMERIC_VALUE,
    /** One more than the last constant for double Unicode properties. @stable ICU 2.4 */
    UCHAR_DOUBLE_LIMIT,

    /** String property Age.
        Corresponds to u_charAge. @stable ICU 2.4 */
    UCHAR_AGE=0x4000,
    /** First constant for string Unicode properties. @stable ICU 2.4 */
    UCHAR_STRING_START=UCHAR_AGE,
    /** String property Bidi_Mirroring_Glyph.
        Corresponds to u_charMirror. @stable ICU 2.4 */
    UCHAR_BIDI_MIRRORING_GLYPH,
    /** String property Case_Folding.
        Corresponds to u_strFoldCase in ustring.h. @stable ICU 2.4 */
    UCHAR_CASE_FOLDING,
    /** String property ISO_Comment.
        Corresponds to u_getISOComment. @stable ICU 2.4 */
    UCHAR_ISO_COMMENT,
    /** String property Lowercase_Mapping.
        Corresponds to u_strToLower in ustring.h. @stable ICU 2.4 */
    UCHAR_LOWERCASE_MAPPING,
    /** String property Name.
        Corresponds to u_charName. @stable ICU 2.4 */
    UCHAR_NAME,
    /** String property Simple_Case_Folding.
        Corresponds to u_foldCase. @stable ICU 2.4 */
    UCHAR_SIMPLE_CASE_FOLDING,
    /** String property Simple_Lowercase_Mapping.
        Corresponds to u_tolower. @stable ICU 2.4 */
    UCHAR_SIMPLE_LOWERCASE_MAPPING,
    /** String property Simple_Titlecase_Mapping.
        Corresponds to u_totitle. @stable ICU 2.4 */
    UCHAR_SIMPLE_TITLECASE_MAPPING,
    /** String property Simple_Uppercase_Mapping.
        Corresponds to u_toupper. @stable ICU 2.4 */
    UCHAR_SIMPLE_UPPERCASE_MAPPING,
    /** String property Titlecase_Mapping.
        Corresponds to u_strToTitle in ustring.h. @stable ICU 2.4 */
    UCHAR_TITLECASE_MAPPING,
    /** String property Unicode_1_Name.
        Corresponds to u_charName. @stable ICU 2.4 */
    UCHAR_UNICODE_1_NAME,
    /** String property Uppercase_Mapping.
        Corresponds to u_strToUpper in ustring.h. @stable ICU 2.4 */
    UCHAR_UPPERCASE_MAPPING,
    /** One more than the last constant for string Unicode properties. @stable ICU 2.4 */
    UCHAR_STRING_LIMIT,

    /** Represents a nonexistent or invalid property or property value. @stable ICU 2.4 */
    UCHAR_INVALID_CODE = -1
} UProperty;

typedef enum UCharCategory
{
    /** See note !!.  Comments of the form "Cn" are read by genpname. */

    /** Non-category for unassigned and non-character code points. @stable ICU 2.0 */
    U_UNASSIGNED              = 0,
    /** Cn "Other, Not Assigned (no characters in [UnicodeData.txt] have this property)" (same as U_UNASSIGNED!) @stable ICU 2.0 */
    U_GENERAL_OTHER_TYPES     = 0,
    /** Lu @stable ICU 2.0 */
    U_UPPERCASE_LETTER        = 1,
    /** Ll @stable ICU 2.0 */
    U_LOWERCASE_LETTER        = 2,
    /** Lt @stable ICU 2.0 */
    U_TITLECASE_LETTER        = 3,
    /** Lm @stable ICU 2.0 */
    U_MODIFIER_LETTER         = 4,
    /** Lo @stable ICU 2.0 */
    U_OTHER_LETTER            = 5,
    /** Mn @stable ICU 2.0 */
    U_NON_SPACING_MARK        = 6,
    /** Me @stable ICU 2.0 */
    U_ENCLOSING_MARK          = 7,
    /** Mc @stable ICU 2.0 */
    U_COMBINING_SPACING_MARK  = 8,
    /** Nd @stable ICU 2.0 */
    U_DECIMAL_DIGIT_NUMBER    = 9,
    /** Nl @stable ICU 2.0 */
    U_LETTER_NUMBER           = 10,
    /** No @stable ICU 2.0 */
    U_OTHER_NUMBER            = 11,
    /** Zs @stable ICU 2.0 */
    U_SPACE_SEPARATOR         = 12,
    /** Zl @stable ICU 2.0 */
    U_LINE_SEPARATOR          = 13,
    /** Zp @stable ICU 2.0 */
    U_PARAGRAPH_SEPARATOR     = 14,
    /** Cc @stable ICU 2.0 */
    U_CONTROL_CHAR            = 15,
    /** Cf @stable ICU 2.0 */
    U_FORMAT_CHAR             = 16,
    /** Co @stable ICU 2.0 */
    U_PRIVATE_USE_CHAR        = 17,
    /** Cs @stable ICU 2.0 */
    U_SURROGATE               = 18,
    /** Pd @stable ICU 2.0 */
    U_DASH_PUNCTUATION        = 19,
    /** Ps @stable ICU 2.0 */
    U_START_PUNCTUATION       = 20,
    /** Pe @stable ICU 2.0 */
    U_END_PUNCTUATION         = 21,
    /** Pc @stable ICU 2.0 */
    U_CONNECTOR_PUNCTUATION   = 22,
    /** Po @stable ICU 2.0 */
    U_OTHER_PUNCTUATION       = 23,
    /** Sm @stable ICU 2.0 */
    U_MATH_SYMBOL             = 24,
    /** Sc @stable ICU 2.0 */
    U_CURRENCY_SYMBOL         = 25,
    /** Sk @stable ICU 2.0 */
    U_MODIFIER_SYMBOL         = 26,
    /** So @stable ICU 2.0 */
    U_OTHER_SYMBOL            = 27,
    /** Pi @stable ICU 2.0 */
    U_INITIAL_PUNCTUATION     = 28,
    /** Pf @stable ICU 2.0 */
    U_FINAL_PUNCTUATION       = 29,
    /** One higher than the last enum UCharCategory constant. @stable ICU 2.0 */
    U_CHAR_CATEGORY_COUNT
} UCharCategory;

#define U_GC_CN_MASK    U_MASK(U_GENERAL_OTHER_TYPES)

/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_LU_MASK    U_MASK(U_UPPERCASE_LETTER)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_LL_MASK    U_MASK(U_LOWERCASE_LETTER)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_LT_MASK    U_MASK(U_TITLECASE_LETTER)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_LM_MASK    U_MASK(U_MODIFIER_LETTER)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_LO_MASK    U_MASK(U_OTHER_LETTER)

/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_MN_MASK    U_MASK(U_NON_SPACING_MARK)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_ME_MASK    U_MASK(U_ENCLOSING_MARK)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_MC_MASK    U_MASK(U_COMBINING_SPACING_MARK)

/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_ND_MASK    U_MASK(U_DECIMAL_DIGIT_NUMBER)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_NL_MASK    U_MASK(U_LETTER_NUMBER)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_NO_MASK    U_MASK(U_OTHER_NUMBER)

/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_ZS_MASK    U_MASK(U_SPACE_SEPARATOR)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_ZL_MASK    U_MASK(U_LINE_SEPARATOR)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_ZP_MASK    U_MASK(U_PARAGRAPH_SEPARATOR)

/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_CC_MASK    U_MASK(U_CONTROL_CHAR)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_CF_MASK    U_MASK(U_FORMAT_CHAR)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_CO_MASK    U_MASK(U_PRIVATE_USE_CHAR)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_CS_MASK    U_MASK(U_SURROGATE)

/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_PD_MASK    U_MASK(U_DASH_PUNCTUATION)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_PS_MASK    U_MASK(U_START_PUNCTUATION)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_PE_MASK    U_MASK(U_END_PUNCTUATION)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_PC_MASK    U_MASK(U_CONNECTOR_PUNCTUATION)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_PO_MASK    U_MASK(U_OTHER_PUNCTUATION)

/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_SM_MASK    U_MASK(U_MATH_SYMBOL)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_SC_MASK    U_MASK(U_CURRENCY_SYMBOL)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_SK_MASK    U_MASK(U_MODIFIER_SYMBOL)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_SO_MASK    U_MASK(U_OTHER_SYMBOL)

/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_PI_MASK    U_MASK(U_INITIAL_PUNCTUATION)
/** Mask constant for a UCharCategory. @stable ICU 2.1 */
#define U_GC_PF_MASK    U_MASK(U_FINAL_PUNCTUATION)


/** Mask constant for multiple UCharCategory bits (L Letters). @stable ICU 2.1 */
#define U_GC_L_MASK \
            (U_GC_LU_MASK|U_GC_LL_MASK|U_GC_LT_MASK|U_GC_LM_MASK|U_GC_LO_MASK)

/** Mask constant for multiple UCharCategory bits (LC Cased Letters). @stable ICU 2.1 */
#define U_GC_LC_MASK \
            (U_GC_LU_MASK|U_GC_LL_MASK|U_GC_LT_MASK)

/** Mask constant for multiple UCharCategory bits (M Marks). @stable ICU 2.1 */
#define U_GC_M_MASK (U_GC_MN_MASK|U_GC_ME_MASK|U_GC_MC_MASK)

/** Mask constant for multiple UCharCategory bits (N Numbers). @stable ICU 2.1 */
#define U_GC_N_MASK (U_GC_ND_MASK|U_GC_NL_MASK|U_GC_NO_MASK)

/** Mask constant for multiple UCharCategory bits (Z Separators). @stable ICU 2.1 */
#define U_GC_Z_MASK (U_GC_ZS_MASK|U_GC_ZL_MASK|U_GC_ZP_MASK)

/** Mask constant for multiple UCharCategory bits (C Others). @stable ICU 2.1 */
#define U_GC_C_MASK \
            (U_GC_CN_MASK|U_GC_CC_MASK|U_GC_CF_MASK|U_GC_CO_MASK|U_GC_CS_MASK)

/** Mask constant for multiple UCharCategory bits (P Punctuation). @stable ICU 2.1 */
#define U_GC_P_MASK \
            (U_GC_PD_MASK|U_GC_PS_MASK|U_GC_PE_MASK|U_GC_PC_MASK|U_GC_PO_MASK| \
             U_GC_PI_MASK|U_GC_PF_MASK)

/** Mask constant for multiple UCharCategory bits (S Symbols). @stable ICU 2.1 */
#define U_GC_S_MASK (U_GC_SM_MASK|U_GC_SC_MASK|U_GC_SK_MASK|U_GC_SO_MASK)

typedef enum UCharDirection {
    /** See note !!.  Comments of the form "EN" are read by genpname. */

    /** L @stable ICU 2.0 */
    U_LEFT_TO_RIGHT               = 0,
    /** R @stable ICU 2.0 */
    U_RIGHT_TO_LEFT               = 1,
    /** EN @stable ICU 2.0 */
    U_EUROPEAN_NUMBER             = 2,
    /** ES @stable ICU 2.0 */
    U_EUROPEAN_NUMBER_SEPARATOR   = 3,
    /** ET @stable ICU 2.0 */
    U_EUROPEAN_NUMBER_TERMINATOR  = 4,
    /** AN @stable ICU 2.0 */
    U_ARABIC_NUMBER               = 5,
    /** CS @stable ICU 2.0 */
    U_COMMON_NUMBER_SEPARATOR     = 6,
    /** B @stable ICU 2.0 */
    U_BLOCK_SEPARATOR             = 7,
    /** S @stable ICU 2.0 */
    U_SEGMENT_SEPARATOR           = 8,
    /** WS @stable ICU 2.0 */
    U_WHITE_SPACE_NEUTRAL         = 9,
    /** ON @stable ICU 2.0 */
    U_OTHER_NEUTRAL               = 10,
    /** LRE @stable ICU 2.0 */
    U_LEFT_TO_RIGHT_EMBEDDING     = 11,
    /** LRO @stable ICU 2.0 */
    U_LEFT_TO_RIGHT_OVERRIDE      = 12,
    /** AL @stable ICU 2.0 */
    U_RIGHT_TO_LEFT_ARABIC        = 13,
    /** RLE @stable ICU 2.0 */
    U_RIGHT_TO_LEFT_EMBEDDING     = 14,
    /** RLO @stable ICU 2.0 */
    U_RIGHT_TO_LEFT_OVERRIDE      = 15,
    /** PDF @stable ICU 2.0 */
    U_POP_DIRECTIONAL_FORMAT      = 16,
    /** NSM @stable ICU 2.0 */
    U_DIR_NON_SPACING_MARK        = 17,
    /** BN @stable ICU 2.0 */
    U_BOUNDARY_NEUTRAL            = 18,
    /** @stable ICU 2.0 */
    U_CHAR_DIRECTION_COUNT
} UCharDirection;

enum UBlockCode {

    /** New No_Block value in Unicode 4. @stable ICU 2.6 */
    UBLOCK_NO_BLOCK = 0, /*[none]*/ /* Special range indicating No_Block */

    /** @stable ICU 2.0 */
    UBLOCK_BASIC_LATIN = 1, /*[0000]*/ /*See note !!*/

    /** @stable ICU 2.0 */
    UBLOCK_LATIN_1_SUPPLEMENT=2, /*[0080]*/

    /** @stable ICU 2.0 */
    UBLOCK_LATIN_EXTENDED_A =3, /*[0100]*/

    /** @stable ICU 2.0 */
    UBLOCK_LATIN_EXTENDED_B =4, /*[0180]*/

    /** @stable ICU 2.0 */
    UBLOCK_IPA_EXTENSIONS =5, /*[0250]*/

    /** @stable ICU 2.0 */
    UBLOCK_SPACING_MODIFIER_LETTERS =6, /*[02B0]*/

    /** @stable ICU 2.0 */
    UBLOCK_COMBINING_DIACRITICAL_MARKS =7, /*[0300]*/

    /**
     * Unicode 3.2 renames this block to "Greek and Coptic".
     * @stable ICU 2.0
     */
    UBLOCK_GREEK =8, /*[0370]*/

    /** @stable ICU 2.0 */
    UBLOCK_CYRILLIC =9, /*[0400]*/

    /** @stable ICU 2.0 */
    UBLOCK_ARMENIAN =10, /*[0530]*/

    /** @stable ICU 2.0 */
    UBLOCK_HEBREW =11, /*[0590]*/

    /** @stable ICU 2.0 */
    UBLOCK_ARABIC =12, /*[0600]*/

    /** @stable ICU 2.0 */
    UBLOCK_SYRIAC =13, /*[0700]*/

    /** @stable ICU 2.0 */
    UBLOCK_THAANA =14, /*[0780]*/

    /** @stable ICU 2.0 */
    UBLOCK_DEVANAGARI =15, /*[0900]*/

    /** @stable ICU 2.0 */
    UBLOCK_BENGALI =16, /*[0980]*/

    /** @stable ICU 2.0 */
    UBLOCK_GURMUKHI =17, /*[0A00]*/

    /** @stable ICU 2.0 */
    UBLOCK_GUJARATI =18, /*[0A80]*/

    /** @stable ICU 2.0 */
    UBLOCK_ORIYA =19, /*[0B00]*/

    /** @stable ICU 2.0 */
    UBLOCK_TAMIL =20, /*[0B80]*/

    /** @stable ICU 2.0 */
    UBLOCK_TELUGU =21, /*[0C00]*/

    /** @stable ICU 2.0 */
    UBLOCK_KANNADA =22, /*[0C80]*/

    /** @stable ICU 2.0 */
    UBLOCK_MALAYALAM =23, /*[0D00]*/

    /** @stable ICU 2.0 */
    UBLOCK_SINHALA =24, /*[0D80]*/

    /** @stable ICU 2.0 */
    UBLOCK_THAI =25, /*[0E00]*/

    /** @stable ICU 2.0 */
    UBLOCK_LAO =26, /*[0E80]*/

    /** @stable ICU 2.0 */
    UBLOCK_TIBETAN =27, /*[0F00]*/

    /** @stable ICU 2.0 */
    UBLOCK_MYANMAR =28, /*[1000]*/

    /** @stable ICU 2.0 */
    UBLOCK_GEORGIAN =29, /*[10A0]*/

    /** @stable ICU 2.0 */
    UBLOCK_HANGUL_JAMO =30, /*[1100]*/

    /** @stable ICU 2.0 */
    UBLOCK_ETHIOPIC =31, /*[1200]*/

    /** @stable ICU 2.0 */
    UBLOCK_CHEROKEE =32, /*[13A0]*/

    /** @stable ICU 2.0 */
    UBLOCK_UNIFIED_CANADIAN_ABORIGINAL_SYLLABICS =33, /*[1400]*/

    /** @stable ICU 2.0 */
    UBLOCK_OGHAM =34, /*[1680]*/

    /** @stable ICU 2.0 */
    UBLOCK_RUNIC =35, /*[16A0]*/

    /** @stable ICU 2.0 */
    UBLOCK_KHMER =36, /*[1780]*/

    /** @stable ICU 2.0 */
    UBLOCK_MONGOLIAN =37, /*[1800]*/

    /** @stable ICU 2.0 */
    UBLOCK_LATIN_EXTENDED_ADDITIONAL =38, /*[1E00]*/

    /** @stable ICU 2.0 */
    UBLOCK_GREEK_EXTENDED =39, /*[1F00]*/

    /** @stable ICU 2.0 */
    UBLOCK_GENERAL_PUNCTUATION =40, /*[2000]*/

    /** @stable ICU 2.0 */
    UBLOCK_SUPERSCRIPTS_AND_SUBSCRIPTS =41, /*[2070]*/

    /** @stable ICU 2.0 */
    UBLOCK_CURRENCY_SYMBOLS =42, /*[20A0]*/

    /**
     * Unicode 3.2 renames this block to "Combining Diacritical Marks for Symbols".
     * @stable ICU 2.0
     */
    UBLOCK_COMBINING_MARKS_FOR_SYMBOLS =43, /*[20D0]*/

    /** @stable ICU 2.0 */
    UBLOCK_LETTERLIKE_SYMBOLS =44, /*[2100]*/

    /** @stable ICU 2.0 */
    UBLOCK_NUMBER_FORMS =45, /*[2150]*/

    /** @stable ICU 2.0 */
    UBLOCK_ARROWS =46, /*[2190]*/

    /** @stable ICU 2.0 */
    UBLOCK_MATHEMATICAL_OPERATORS =47, /*[2200]*/

    /** @stable ICU 2.0 */
    UBLOCK_MISCELLANEOUS_TECHNICAL =48, /*[2300]*/

    /** @stable ICU 2.0 */
    UBLOCK_CONTROL_PICTURES =49, /*[2400]*/

    /** @stable ICU 2.0 */
    UBLOCK_OPTICAL_CHARACTER_RECOGNITION =50, /*[2440]*/

    /** @stable ICU 2.0 */
    UBLOCK_ENCLOSED_ALPHANUMERICS =51, /*[2460]*/

    /** @stable ICU 2.0 */
    UBLOCK_BOX_DRAWING =52, /*[2500]*/

    /** @stable ICU 2.0 */
    UBLOCK_BLOCK_ELEMENTS =53, /*[2580]*/

    /** @stable ICU 2.0 */
    UBLOCK_GEOMETRIC_SHAPES =54, /*[25A0]*/

    /** @stable ICU 2.0 */
    UBLOCK_MISCELLANEOUS_SYMBOLS =55, /*[2600]*/

    /** @stable ICU 2.0 */
    UBLOCK_DINGBATS =56, /*[2700]*/

    /** @stable ICU 2.0 */
    UBLOCK_BRAILLE_PATTERNS =57, /*[2800]*/

    /** @stable ICU 2.0 */
    UBLOCK_CJK_RADICALS_SUPPLEMENT =58, /*[2E80]*/

    /** @stable ICU 2.0 */
    UBLOCK_KANGXI_RADICALS =59, /*[2F00]*/

    /** @stable ICU 2.0 */
    UBLOCK_IDEOGRAPHIC_DESCRIPTION_CHARACTERS =60, /*[2FF0]*/

    /** @stable ICU 2.0 */
    UBLOCK_CJK_SYMBOLS_AND_PUNCTUATION =61, /*[3000]*/

    /** @stable ICU 2.0 */
    UBLOCK_HIRAGANA =62, /*[3040]*/

    /** @stable ICU 2.0 */
    UBLOCK_KATAKANA =63, /*[30A0]*/

    /** @stable ICU 2.0 */
    UBLOCK_BOPOMOFO =64, /*[3100]*/

    /** @stable ICU 2.0 */
    UBLOCK_HANGUL_COMPATIBILITY_JAMO =65, /*[3130]*/

    /** @stable ICU 2.0 */
    UBLOCK_KANBUN =66, /*[3190]*/

    /** @stable ICU 2.0 */
    UBLOCK_BOPOMOFO_EXTENDED =67, /*[31A0]*/

    /** @stable ICU 2.0 */
    UBLOCK_ENCLOSED_CJK_LETTERS_AND_MONTHS =68, /*[3200]*/

    /** @stable ICU 2.0 */
    UBLOCK_CJK_COMPATIBILITY =69, /*[3300]*/

    /** @stable ICU 2.0 */
    UBLOCK_CJK_UNIFIED_IDEOGRAPHS_EXTENSION_A =70, /*[3400]*/

    /** @stable ICU 2.0 */
    UBLOCK_CJK_UNIFIED_IDEOGRAPHS =71, /*[4E00]*/

    /** @stable ICU 2.0 */
    UBLOCK_YI_SYLLABLES =72, /*[A000]*/

    /** @stable ICU 2.0 */
    UBLOCK_YI_RADICALS =73, /*[A490]*/

    /** @stable ICU 2.0 */
    UBLOCK_HANGUL_SYLLABLES =74, /*[AC00]*/

    /** @stable ICU 2.0 */
    UBLOCK_HIGH_SURROGATES =75, /*[D800]*/

    /** @stable ICU 2.0 */
    UBLOCK_HIGH_PRIVATE_USE_SURROGATES =76, /*[DB80]*/

    /** @stable ICU 2.0 */
    UBLOCK_LOW_SURROGATES =77, /*[DC00]*/

    /**
     * Same as UBLOCK_PRIVATE_USE_AREA.
     * Until Unicode 3.1.1, the corresponding block name was "Private Use",
     * and multiple code point ranges had this block.
     * Unicode 3.2 renames the block for the BMP PUA to "Private Use Area" and
     * adds separate blocks for the supplementary PUAs.
     *
     * @stable ICU 2.0
     */
    UBLOCK_PRIVATE_USE = 78,
    /**
     * Same as UBLOCK_PRIVATE_USE.
     * Until Unicode 3.1.1, the corresponding block name was "Private Use",
     * and multiple code point ranges had this block.
     * Unicode 3.2 renames the block for the BMP PUA to "Private Use Area" and
     * adds separate blocks for the supplementary PUAs.
     *
     * @stable ICU 2.0
     */
    UBLOCK_PRIVATE_USE_AREA =UBLOCK_PRIVATE_USE, /*[E000]*/

    /** @stable ICU 2.0 */
    UBLOCK_CJK_COMPATIBILITY_IDEOGRAPHS =79, /*[F900]*/

    /** @stable ICU 2.0 */
    UBLOCK_ALPHABETIC_PRESENTATION_FORMS =80, /*[FB00]*/

    /** @stable ICU 2.0 */
    UBLOCK_ARABIC_PRESENTATION_FORMS_A =81, /*[FB50]*/

    /** @stable ICU 2.0 */
    UBLOCK_COMBINING_HALF_MARKS =82, /*[FE20]*/

    /** @stable ICU 2.0 */
    UBLOCK_CJK_COMPATIBILITY_FORMS =83, /*[FE30]*/

    /** @stable ICU 2.0 */
    UBLOCK_SMALL_FORM_VARIANTS =84, /*[FE50]*/

    /** @stable ICU 2.0 */
    UBLOCK_ARABIC_PRESENTATION_FORMS_B =85, /*[FE70]*/

    /** @stable ICU 2.0 */
    UBLOCK_SPECIALS =86, /*[FFF0]*/

    /** @stable ICU 2.0 */
    UBLOCK_HALFWIDTH_AND_FULLWIDTH_FORMS =87, /*[FF00]*/

    /* New blocks in Unicode 3.1 */

    /** @stable ICU 2.0 */
    UBLOCK_OLD_ITALIC = 88  , /*[10300]*/
    /** @stable ICU 2.0 */
    UBLOCK_GOTHIC = 89 , /*[10330]*/
    /** @stable ICU 2.0 */
    UBLOCK_DESERET = 90 , /*[10400]*/
    /** @stable ICU 2.0 */
    UBLOCK_BYZANTINE_MUSICAL_SYMBOLS = 91 , /*[1D000]*/
    /** @stable ICU 2.0 */
    UBLOCK_MUSICAL_SYMBOLS = 92 , /*[1D100]*/
    /** @stable ICU 2.0 */
    UBLOCK_MATHEMATICAL_ALPHANUMERIC_SYMBOLS = 93  , /*[1D400]*/
    /** @stable ICU 2.0 */
    UBLOCK_CJK_UNIFIED_IDEOGRAPHS_EXTENSION_B  = 94 , /*[20000]*/
    /** @stable ICU 2.0 */
    UBLOCK_CJK_COMPATIBILITY_IDEOGRAPHS_SUPPLEMENT = 95 , /*[2F800]*/
    /** @stable ICU 2.0 */
    UBLOCK_TAGS = 96, /*[E0000]*/

    /* New blocks in Unicode 3.2 */

    /**
     * Unicode 4.0.1 renames the "Cyrillic Supplementary" block to "Cyrillic Supplement".
     * @stable ICU 2.2
     */
    UBLOCK_CYRILLIC_SUPPLEMENTARY = 97, 
    /** @draft ICU 3.0  */
    UBLOCK_CYRILLIC_SUPPLEMENT = UBLOCK_CYRILLIC_SUPPLEMENTARY, /*[0500]*/
    /** @stable ICU 2.2 */
    UBLOCK_TAGALOG = 98, /*[1700]*/
    /** @stable ICU 2.2 */
    UBLOCK_HANUNOO = 99, /*[1720]*/
    /** @stable ICU 2.2 */
    UBLOCK_BUHID = 100, /*[1740]*/
    /** @stable ICU 2.2 */
    UBLOCK_TAGBANWA = 101, /*[1760]*/
    /** @stable ICU 2.2 */
    UBLOCK_MISCELLANEOUS_MATHEMATICAL_SYMBOLS_A = 102, /*[27C0]*/
    /** @stable ICU 2.2 */
    UBLOCK_SUPPLEMENTAL_ARROWS_A = 103, /*[27F0]*/
    /** @stable ICU 2.2 */
    UBLOCK_SUPPLEMENTAL_ARROWS_B = 104, /*[2900]*/
    /** @stable ICU 2.2 */
    UBLOCK_MISCELLANEOUS_MATHEMATICAL_SYMBOLS_B = 105, /*[2980]*/
    /** @stable ICU 2.2 */
    UBLOCK_SUPPLEMENTAL_MATHEMATICAL_OPERATORS = 106, /*[2A00]*/
    /** @stable ICU 2.2 */
    UBLOCK_KATAKANA_PHONETIC_EXTENSIONS = 107, /*[31F0]*/
    /** @stable ICU 2.2 */
    UBLOCK_VARIATION_SELECTORS = 108, /*[FE00]*/
    /** @stable ICU 2.2 */
    UBLOCK_SUPPLEMENTARY_PRIVATE_USE_AREA_A = 109, /*[F0000]*/
    /** @stable ICU 2.2 */
    UBLOCK_SUPPLEMENTARY_PRIVATE_USE_AREA_B = 110, /*[100000]*/

    /* New blocks in Unicode 4 */

    /** @stable ICU 2.6 */
    UBLOCK_LIMBU = 111, /*[1900]*/
    /** @stable ICU 2.6 */
    UBLOCK_TAI_LE = 112, /*[1950]*/
    /** @stable ICU 2.6 */
    UBLOCK_KHMER_SYMBOLS = 113, /*[19E0]*/
    /** @stable ICU 2.6 */
    UBLOCK_PHONETIC_EXTENSIONS = 114, /*[1D00]*/
    /** @stable ICU 2.6 */
    UBLOCK_MISCELLANEOUS_SYMBOLS_AND_ARROWS = 115, /*[2B00]*/
    /** @stable ICU 2.6 */
    UBLOCK_YIJING_HEXAGRAM_SYMBOLS = 116, /*[4DC0]*/
    /** @stable ICU 2.6 */
    UBLOCK_LINEAR_B_SYLLABARY = 117, /*[10000]*/
    /** @stable ICU 2.6 */
    UBLOCK_LINEAR_B_IDEOGRAMS = 118, /*[10080]*/
    /** @stable ICU 2.6 */
    UBLOCK_AEGEAN_NUMBERS = 119, /*[10100]*/
    /** @stable ICU 2.6 */
    UBLOCK_UGARITIC = 120, /*[10380]*/
    /** @stable ICU 2.6 */
    UBLOCK_SHAVIAN = 121, /*[10450]*/
    /** @stable ICU 2.6 */
    UBLOCK_OSMANYA = 122, /*[10480]*/
    /** @stable ICU 2.6 */
    UBLOCK_CYPRIOT_SYLLABARY = 123, /*[10800]*/
    /** @stable ICU 2.6 */
    UBLOCK_TAI_XUAN_JING_SYMBOLS = 124, /*[1D300]*/
    /** @stable ICU 2.6 */
    UBLOCK_VARIATION_SELECTORS_SUPPLEMENT = 125, /*[E0100]*/

    /** @stable ICU 2.0 */
    UBLOCK_COUNT,

    /** @stable ICU 2.0 */
    UBLOCK_INVALID_CODE=-1
};

/** @stable ICU 2.0 */
typedef enum UBlockCode UBlockCode;

typedef enum UEastAsianWidth {
    U_EA_NEUTRAL,   /*[N]*/ /*See note !!*/
    U_EA_AMBIGUOUS, /*[A]*/
    U_EA_HALFWIDTH, /*[H]*/
    U_EA_FULLWIDTH, /*[F]*/
    U_EA_NARROW,    /*[Na]*/
    U_EA_WIDE,      /*[W]*/
    U_EA_COUNT
} UEastAsianWidth;

typedef enum UCharNameChoice {
    U_UNICODE_CHAR_NAME,
    U_UNICODE_10_CHAR_NAME,
    U_EXTENDED_CHAR_NAME,
    U_CHAR_NAME_CHOICE_COUNT
} UCharNameChoice;

typedef enum UPropertyNameChoice {
    U_SHORT_PROPERTY_NAME,
    U_LONG_PROPERTY_NAME,
    U_PROPERTY_NAME_CHOICE_COUNT
} UPropertyNameChoice;

typedef enum UDecompositionType {
    U_DT_NONE,              /*[none]*/ /*See note !!*/
    U_DT_CANONICAL,         /*[can]*/
    U_DT_COMPAT,            /*[com]*/
    U_DT_CIRCLE,            /*[enc]*/
    U_DT_FINAL,             /*[fin]*/
    U_DT_FONT,              /*[font]*/
    U_DT_FRACTION,          /*[fra]*/
    U_DT_INITIAL,           /*[init]*/
    U_DT_ISOLATED,          /*[iso]*/
    U_DT_MEDIAL,            /*[med]*/
    U_DT_NARROW,            /*[nar]*/
    U_DT_NOBREAK,           /*[nb]*/
    U_DT_SMALL,             /*[sml]*/
    U_DT_SQUARE,            /*[sqr]*/
    U_DT_SUB,               /*[sub]*/
    U_DT_SUPER,             /*[sup]*/
    U_DT_VERTICAL,          /*[vert]*/
    U_DT_WIDE,              /*[wide]*/
    U_DT_COUNT /* 18 */
} UDecompositionType;

typedef enum UJoiningType {
    U_JT_NON_JOINING,       /*[U]*/ /*See note !!*/
    U_JT_JOIN_CAUSING,      /*[C]*/
    U_JT_DUAL_JOINING,      /*[D]*/
    U_JT_LEFT_JOINING,      /*[L]*/
    U_JT_RIGHT_JOINING,     /*[R]*/
    U_JT_TRANSPARENT,       /*[T]*/
    U_JT_COUNT /* 6 */
} UJoiningType;

typedef enum UJoiningGroup {
    U_JG_NO_JOINING_GROUP,
    U_JG_AIN,
    U_JG_ALAPH,
    U_JG_ALEF,
    U_JG_BEH,
    U_JG_BETH,
    U_JG_DAL,
    U_JG_DALATH_RISH,
    U_JG_E,
    U_JG_FEH,
    U_JG_FINAL_SEMKATH,
    U_JG_GAF,
    U_JG_GAMAL,
    U_JG_HAH,
    U_JG_HAMZA_ON_HEH_GOAL,
    U_JG_HE,
    U_JG_HEH,
    U_JG_HEH_GOAL,
    U_JG_HETH,
    U_JG_KAF,
    U_JG_KAPH,
    U_JG_KNOTTED_HEH,
    U_JG_LAM,
    U_JG_LAMADH,
    U_JG_MEEM,
    U_JG_MIM,
    U_JG_NOON,
    U_JG_NUN,
    U_JG_PE,
    U_JG_QAF,
    U_JG_QAPH,
    U_JG_REH,
    U_JG_REVERSED_PE,
    U_JG_SAD,
    U_JG_SADHE,
    U_JG_SEEN,
    U_JG_SEMKATH,
    U_JG_SHIN,
    U_JG_SWASH_KAF,
    U_JG_SYRIAC_WAW,
    U_JG_TAH,
    U_JG_TAW,
    U_JG_TEH_MARBUTA,
    U_JG_TETH,
    U_JG_WAW,
    U_JG_YEH,
    U_JG_YEH_BARREE,
    U_JG_YEH_WITH_TAIL,
    U_JG_YUDH,
    U_JG_YUDH_HE,
    U_JG_ZAIN,
    U_JG_FE,        /**< @stable ICU 2.6 */
    U_JG_KHAPH,     /**< @stable ICU 2.6 */
    U_JG_ZHAIN,     /**< @stable ICU 2.6 */
    U_JG_COUNT
} UJoiningGroup;

typedef enum ULineBreak {
    U_LB_UNKNOWN,           /*[XX]*/ /*See note !!*/
    U_LB_AMBIGUOUS,         /*[AI]*/
    U_LB_ALPHABETIC,        /*[AL]*/
    U_LB_BREAK_BOTH,        /*[B2]*/
    U_LB_BREAK_AFTER,       /*[BA]*/
    U_LB_BREAK_BEFORE,      /*[BB]*/
    U_LB_MANDATORY_BREAK,   /*[BK]*/
    U_LB_CONTINGENT_BREAK,  /*[CB]*/
    U_LB_CLOSE_PUNCTUATION, /*[CL]*/
    U_LB_COMBINING_MARK,    /*[CM]*/
    U_LB_CARRIAGE_RETURN,   /*[CR]*/
    U_LB_EXCLAMATION,       /*[EX]*/
    U_LB_GLUE,              /*[GL]*/
    U_LB_HYPHEN,            /*[HY]*/
    U_LB_IDEOGRAPHIC,       /*[ID]*/
    U_LB_INSEPERABLE,
    /** Renamed from the misspelled "inseperable" in Unicode 4.0.1/ICU 3.0 @draft ICU 3.0 */
    U_LB_INSEPARABLE=U_LB_INSEPERABLE,/*[IN]*/
    U_LB_INFIX_NUMERIC,     /*[IS]*/
    U_LB_LINE_FEED,         /*[LF]*/
    U_LB_NONSTARTER,        /*[NS]*/
    U_LB_NUMERIC,           /*[NU]*/
    U_LB_OPEN_PUNCTUATION,  /*[OP]*/
    U_LB_POSTFIX_NUMERIC,   /*[PO]*/
    U_LB_PREFIX_NUMERIC,    /*[PR]*/
    U_LB_QUOTATION,         /*[QU]*/
    U_LB_COMPLEX_CONTEXT,   /*[SA]*/
    U_LB_SURROGATE,         /*[SG]*/
    U_LB_SPACE,             /*[SP]*/
    U_LB_BREAK_SYMBOLS,     /*[SY]*/
    U_LB_ZWSPACE,           /*[ZW]*/
    U_LB_NEXT_LINE,         /*[NL]*/ /* from here on: new in Unicode 4/ICU 2.6 */
    U_LB_WORD_JOINER,       /*[WJ]*/
    U_LB_COUNT
} ULineBreak;

typedef enum UNumericType {
    U_NT_NONE,              /*[None]*/ /*See note !!*/
    U_NT_DECIMAL,           /*[de]*/
    U_NT_DIGIT,             /*[di]*/
    U_NT_NUMERIC,           /*[nu]*/
    U_NT_COUNT
} UNumericType;

typedef enum UHangulSyllableType {
    U_HST_NOT_APPLICABLE,   /*[NA]*/ /*See note !!*/
    U_HST_LEADING_JAMO,     /*[L]*/
    U_HST_VOWEL_JAMO,       /*[V]*/
    U_HST_TRAILING_JAMO,    /*[T]*/
    U_HST_LV_SYLLABLE,      /*[LV]*/
    U_HST_LVT_SYLLABLE,     /*[LVT]*/
    U_HST_COUNT
} UHangulSyllableType;

U_STABLE UBool U_EXPORT2
u_hasBinaryProperty(UChar32 c, UProperty which);

U_STABLE UBool U_EXPORT2
u_isUAlphabetic(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isULowercase(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isUUppercase(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isUWhiteSpace(UChar32 c);

U_STABLE int32_t U_EXPORT2
u_getIntPropertyValue(UChar32 c, UProperty which);

U_STABLE int32_t U_EXPORT2
u_getIntPropertyMinValue(UProperty which);

U_STABLE int32_t U_EXPORT2
u_getIntPropertyMaxValue(UProperty which);

U_STABLE double U_EXPORT2
u_getNumericValue(UChar32 c);

#define U_NO_NUMERIC_VALUE ((double)-123456789.)

U_STABLE UBool U_EXPORT2
u_islower(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isupper(UChar32 c);

U_STABLE UBool U_EXPORT2
u_istitle(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isdigit(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isalpha(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isalnum(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isxdigit(UChar32 c);

U_STABLE UBool U_EXPORT2
u_ispunct(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isgraph(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isblank(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isdefined(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isspace(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isJavaSpaceChar(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isWhitespace(UChar32 c);

U_STABLE UBool U_EXPORT2
u_iscntrl(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isISOControl(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isprint(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isbase(UChar32 c);

U_STABLE UCharDirection U_EXPORT2
u_charDirection(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isMirrored(UChar32 c);

U_STABLE UChar32 U_EXPORT2
u_charMirror(UChar32 c);

U_STABLE int8_t U_EXPORT2
u_charType(UChar32 c);

#define U_GET_GC_MASK(c) U_MASK(u_charType(c))

typedef UBool U_CALLCONV
UCharEnumTypeRange(const void *context, UChar32 start, UChar32 limit, UCharCategory type);

U_STABLE void U_EXPORT2
u_enumCharTypes(UCharEnumTypeRange *enumRange, const void *context);

#if !UCONFIG_NO_NORMALIZATION

U_STABLE uint8_t U_EXPORT2
u_getCombiningClass(UChar32 c);

#endif

U_STABLE int32_t U_EXPORT2
u_charDigitValue(UChar32 c);

U_STABLE UBlockCode U_EXPORT2
ublock_getCode(UChar32 c);

U_STABLE int32_t U_EXPORT2
u_charName(UChar32 code, UCharNameChoice nameChoice,
           char *buffer, int32_t bufferLength,
           UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
u_getISOComment(UChar32 c,
                char *dest, int32_t destCapacity,
                UErrorCode *pErrorCode);

U_STABLE UChar32 U_EXPORT2
u_charFromName(UCharNameChoice nameChoice,
               const char *name,
               UErrorCode *pErrorCode);

typedef UBool UEnumCharNamesFn(void *context,
                               UChar32 code,
                               UCharNameChoice nameChoice,
                               const char *name,
                               int32_t length);

U_STABLE void U_EXPORT2
u_enumCharNames(UChar32 start, UChar32 limit,
                UEnumCharNamesFn *fn,
                void *context,
                UCharNameChoice nameChoice,
                UErrorCode *pErrorCode);

U_STABLE const char* U_EXPORT2
u_getPropertyName(UProperty property,
                  UPropertyNameChoice nameChoice);

U_STABLE UProperty U_EXPORT2
u_getPropertyEnum(const char* alias);

U_STABLE const char* U_EXPORT2
u_getPropertyValueName(UProperty property,
                       int32_t value,
                       UPropertyNameChoice nameChoice);

U_STABLE int32_t U_EXPORT2
u_getPropertyValueEnum(UProperty property,
                       const char* alias);

U_STABLE UBool U_EXPORT2
u_isIDStart(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isIDPart(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isIDIgnorable(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isJavaIDStart(UChar32 c);

U_STABLE UBool U_EXPORT2
u_isJavaIDPart(UChar32 c);

U_STABLE UChar32 U_EXPORT2
u_tolower(UChar32 c);

U_STABLE UChar32 U_EXPORT2
u_toupper(UChar32 c);

U_STABLE UChar32 U_EXPORT2
u_totitle(UChar32 c);

/** Option value for case folding: use default mappings defined in CaseFolding.txt. @stable ICU 2.0 */
#define U_FOLD_CASE_DEFAULT 0

#define U_FOLD_CASE_EXCLUDE_SPECIAL_I 1

U_STABLE UChar32 U_EXPORT2
u_foldCase(UChar32 c, uint32_t options);

U_STABLE int32_t U_EXPORT2
u_digit(UChar32 ch, int8_t radix);

U_STABLE UChar32 U_EXPORT2
u_forDigit(int32_t digit, int8_t radix);

U_STABLE void U_EXPORT2
u_charAge(UChar32 c, UVersionInfo versionArray);

U_STABLE void U_EXPORT2
u_getUnicodeVersion(UVersionInfo versionArray);

U_STABLE int32_t U_EXPORT2
u_getFC_NFKC_Closure(UChar32 c, UChar *dest, int32_t destCapacity, UErrorCode *pErrorCode);

U_CDECL_END

#endif /*_UCHAR*/
/*eof*/
