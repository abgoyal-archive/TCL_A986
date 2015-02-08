
#ifndef __UCASE_H__
#define __UCASE_H__

#include "unicode/utypes.h"
#include "unicode/uset.h"
#include "uset_imp.h"
#include "udataswp.h"

U_CDECL_BEGIN

/* library API -------------------------------------------------------------- */

struct UCaseProps;
typedef struct UCaseProps UCaseProps;

U_CAPI UCaseProps * U_EXPORT2
ucase_open(UErrorCode *pErrorCode);

U_CAPI UCaseProps * U_EXPORT2
ucase_openBinary(const uint8_t *bin, int32_t length, UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
ucase_close(UCaseProps *csp);


U_CAPI const UCaseProps * U_EXPORT2
ucase_getSingleton(UErrorCode *pErrorCode);

#define UCASE_HARDCODE_DATA 1

#if !UCASE_HARDCODE_DATA
U_CAPI const UCaseProps * U_EXPORT2
ucase_getDummy(UErrorCode *pErrorCode);
#endif


U_CAPI int32_t U_EXPORT2
ucase_swap(const UDataSwapper *ds,
           const void *inData, int32_t length, void *outData,
           UErrorCode *pErrorCode);

U_CFUNC void U_EXPORT2
ucase_addPropertyStarts(const UCaseProps *csp, const USetAdder *sa, UErrorCode *pErrorCode);

U_CFUNC int32_t
ucase_getCaseLocale(const char *locale, int32_t *locCache);

/* Casing locale types for ucase_getCaseLocale */
enum {
    UCASE_LOC_UNKNOWN,
    UCASE_LOC_ROOT,
    UCASE_LOC_TURKISH,
    UCASE_LOC_LITHUANIAN,
    UCASE_LOC_DUTCH
};

#define _STRCASECMP_OPTIONS_MASK 0xffff

#define _FOLD_CASE_OPTIONS_MASK 0xff

/* single-code point functions */

U_CAPI UChar32 U_EXPORT2
ucase_tolower(const UCaseProps *csp, UChar32 c);

U_CAPI UChar32 U_EXPORT2
ucase_toupper(const UCaseProps *csp, UChar32 c);

U_CAPI UChar32 U_EXPORT2
ucase_totitle(const UCaseProps *csp, UChar32 c);

U_CAPI UChar32 U_EXPORT2
ucase_fold(const UCaseProps *csp, UChar32 c, uint32_t options);

U_CFUNC void U_EXPORT2
ucase_addCaseClosure(const UCaseProps *csp, UChar32 c, const USetAdder *sa);

U_CFUNC UBool U_EXPORT2
ucase_addStringCaseClosure(const UCaseProps *csp, const UChar *s, int32_t length, const USetAdder *sa);

/** @return UCASE_NONE, UCASE_LOWER, UCASE_UPPER, UCASE_TITLE */
U_CAPI int32_t U_EXPORT2
ucase_getType(const UCaseProps *csp, UChar32 c);

/** @return same as ucase_getType(), or <0 if c is case-ignorable */
U_CAPI int32_t U_EXPORT2
ucase_getTypeOrIgnorable(const UCaseProps *csp, UChar32 c);

U_CAPI UBool U_EXPORT2
ucase_isSoftDotted(const UCaseProps *csp, UChar32 c);

U_CAPI UBool U_EXPORT2
ucase_isCaseSensitive(const UCaseProps *csp, UChar32 c);

/* string case mapping functions */

typedef UChar32 U_CALLCONV
UCaseContextIterator(void *context, int8_t dir);

struct UCaseContext {
    void *p;
    int32_t start, index, limit;
    int32_t cpStart, cpLimit;
    int8_t dir;
    int8_t b1, b2, b3;
};
typedef struct UCaseContext UCaseContext;

enum {
    /**
     * For string case mappings, a single character (a code point) is mapped
     * either to itself (in which case in-place mapping functions do nothing),
     * or to another single code point, or to a string.
     * Aside from the string contents, these are indicated with a single int32_t
     * value as follows:
     *
     * Mapping to self: Negative values (~self instead of -self to support U+0000)
     *
     * Mapping to another code point: Positive values >UCASE_MAX_STRING_LENGTH
     *
     * Mapping to a string: The string length (0..UCASE_MAX_STRING_LENGTH) is
     * returned. Note that the string result may indeed have zero length.
     */
    UCASE_MAX_STRING_LENGTH=0x1f
};

U_CAPI int32_t U_EXPORT2
ucase_toFullLower(const UCaseProps *csp, UChar32 c,
                  UCaseContextIterator *iter, void *context,
                  const UChar **pString,
                  const char *locale, int32_t *locCache);

U_CAPI int32_t U_EXPORT2
ucase_toFullUpper(const UCaseProps *csp, UChar32 c,
                  UCaseContextIterator *iter, void *context,
                  const UChar **pString,
                  const char *locale, int32_t *locCache);

U_CAPI int32_t U_EXPORT2
ucase_toFullTitle(const UCaseProps *csp, UChar32 c,
                  UCaseContextIterator *iter, void *context,
                  const UChar **pString,
                  const char *locale, int32_t *locCache);

U_CAPI int32_t U_EXPORT2
ucase_toFullFolding(const UCaseProps *csp, UChar32 c,
                    const UChar **pString,
                    uint32_t options);

U_CFUNC int32_t U_EXPORT2
ucase_hasBinaryProperty(UChar32 c, UProperty which);


U_CDECL_BEGIN

typedef int32_t U_CALLCONV
UCaseMapFull(const UCaseProps *csp, UChar32 c,
             UCaseContextIterator *iter, void *context,
             const UChar **pString,
             const char *locale, int32_t *locCache);

U_CDECL_END

/* file definitions --------------------------------------------------------- */

#define UCASE_DATA_NAME "ucase"
#define UCASE_DATA_TYPE "icu"

/* format "cAsE" */
#define UCASE_FMT_0 0x63
#define UCASE_FMT_1 0x41
#define UCASE_FMT_2 0x53
#define UCASE_FMT_3 0x45

/* indexes into indexes[] */
enum {
    UCASE_IX_INDEX_TOP,
    UCASE_IX_LENGTH,
    UCASE_IX_TRIE_SIZE,
    UCASE_IX_EXC_LENGTH,
    UCASE_IX_UNFOLD_LENGTH,

    UCASE_IX_MAX_FULL_LENGTH=15,
    UCASE_IX_TOP=16
};

/* definitions for 16-bit case properties word ------------------------------ */

/* 2-bit constants for types of cased characters */
#define UCASE_TYPE_MASK     3
enum {
    UCASE_NONE,
    UCASE_LOWER,
    UCASE_UPPER,
    UCASE_TITLE
};

#define UCASE_GET_TYPE(props) ((props)&UCASE_TYPE_MASK)

#define UCASE_SENSITIVE     4
#define UCASE_EXCEPTION     8

#define UCASE_DOT_MASK      0x30
enum {
    UCASE_NO_DOT=0,         /* normal characters with cc=0 */
    UCASE_SOFT_DOTTED=0x10, /* soft-dotted characters with cc=0 */
    UCASE_ABOVE=0x20,       /* "above" accents with cc=230 */
    UCASE_OTHER_ACCENT=0x30 /* other accent character (0<cc!=230) */
};

/* no exception: bits 15..6 are a 10-bit signed case mapping delta */
#define UCASE_DELTA_SHIFT   6
#define UCASE_DELTA_MASK    0xffc0
#define UCASE_MAX_DELTA     0x1ff
#define UCASE_MIN_DELTA     (-UCASE_MAX_DELTA-1)

#define UCASE_GET_DELTA(props) ((int16_t)(props)>>UCASE_DELTA_SHIFT)

/* case-ignorable uses one of the delta bits, see gencase/store.c */
#define UCASE_CASE_IGNORABLE 0x40

/* exception: bits 15..4 are an unsigned 12-bit index into the exceptions array */
#define UCASE_EXC_SHIFT     4
#define UCASE_EXC_MASK      0xfff0
#define UCASE_MAX_EXCEPTIONS 0x1000

/* definitions for 16-bit main exceptions word ------------------------------ */

/* first 8 bits indicate values in optional slots */
enum {
    UCASE_EXC_LOWER,
    UCASE_EXC_FOLD,
    UCASE_EXC_UPPER,
    UCASE_EXC_TITLE,
    UCASE_EXC_4,            /* reserved */
    UCASE_EXC_5,            /* reserved */
    UCASE_EXC_CLOSURE,
    UCASE_EXC_FULL_MAPPINGS,
    UCASE_EXC_ALL_SLOTS     /* one past the last slot */
};

/* each slot is 2 uint16_t instead of 1 */
#define UCASE_EXC_DOUBLE_SLOTS      0x100

/* reserved: exception bits 10..9 */

#define UCASE_EXC_CASE_IGNORABLE        0x800

/* UCASE_EXC_DOT_MASK=UCASE_DOT_MASK<<UCASE_EXC_DOT_SHIFT */
#define UCASE_EXC_DOT_SHIFT     8

/* normally stored in the main word, but pushed out for larger exception indexes */
#define UCASE_EXC_DOT_MASK      0x3000
enum {
    UCASE_EXC_NO_DOT=0,
    UCASE_EXC_SOFT_DOTTED=0x1000,
    UCASE_EXC_ABOVE=0x2000,         /* "above" accents with cc=230 */
    UCASE_EXC_OTHER_ACCENT=0x3000   /* other character (0<cc!=230) */
};

/* complex/conditional mappings */
#define UCASE_EXC_CONDITIONAL_SPECIAL   0x4000
#define UCASE_EXC_CONDITIONAL_FOLD      0x8000

/* definitions for lengths word for full case mappings */
#define UCASE_FULL_LOWER    0xf
#define UCASE_FULL_FOLDING  0xf0
#define UCASE_FULL_UPPER    0xf00
#define UCASE_FULL_TITLE    0xf000

/* maximum lengths */
#define UCASE_FULL_MAPPINGS_MAX_LENGTH (4*0xf)
#define UCASE_CLOSURE_MAX_LENGTH 0xf

/* constants for reverse case folding ("unfold") data */
enum {
    UCASE_UNFOLD_ROWS,
    UCASE_UNFOLD_ROW_WIDTH,
    UCASE_UNFOLD_STRING_WIDTH
};

U_CDECL_END

#endif
