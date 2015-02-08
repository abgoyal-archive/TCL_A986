


#ifndef PCRE_INTERNAL_H
#define PCRE_INTERNAL_H

/* Bit definitions for entries in the pcre_ctypes table. */

#define ctype_space   0x01
#define ctype_xdigit  0x08
#define ctype_word    0x10   /* alphameric or '_' */


#define cbit_space     0      /* \s */
#define cbit_digit    32      /* \d */
#define cbit_word     64      /* \w */
#define cbit_length   96      /* Length of the cbits table */


#define lcc_offset      0
#define fcc_offset    128
#define cbits_offset  256
#define ctypes_offset (cbits_offset + cbit_length)
#define tables_length (ctypes_offset + 128)

#ifndef DFTABLES

// Change the following to 1 to dump used regular expressions at process exit time.
#define REGEXP_HISTOGRAM 0

#include "Assertions.h"

#if COMPILER(MSVC)
#pragma warning(disable: 4232)
#pragma warning(disable: 4244)
#endif

#include "pcre.h"


#define LINK_SIZE   3

/* Define DEBUG to get debugging output on stdout. */

#if 0
#define DEBUG
#endif


#ifdef DEBUG
#define DPRINTF(p) printf p
#else
#define DPRINTF(p) /*nothing*/
#endif



static inline void put2ByteValue(unsigned char* opcodePtr, int value)
{
    ASSERT(value >= 0 && value <= 0xFFFF);
    opcodePtr[0] = value >> 8;
    opcodePtr[1] = value;
}

static inline void put3ByteValue(unsigned char* opcodePtr, int value)
{
    ASSERT(value >= 0 && value <= 0xFFFFFF);
    opcodePtr[0] = value >> 16;
    opcodePtr[1] = value >> 8;
    opcodePtr[2] = value;
}

static inline int get2ByteValue(const unsigned char* opcodePtr)
{
    return (opcodePtr[0] << 8) | opcodePtr[1];
}

static inline int get3ByteValue(const unsigned char* opcodePtr)
{
    return (opcodePtr[0] << 16) | (opcodePtr[1] << 8) | opcodePtr[2];
}

static inline void put2ByteValueAndAdvance(unsigned char*& opcodePtr, int value)
{
    put2ByteValue(opcodePtr, value);
    opcodePtr += 2;
}

static inline void put3ByteValueAndAdvance(unsigned char*& opcodePtr, int value)
{
    put3ByteValue(opcodePtr, value);
    opcodePtr += 3;
}

static inline void putLinkValueAllowZero(unsigned char* opcodePtr, int value)
{
#if LINK_SIZE == 3
    put3ByteValue(opcodePtr, value);
#elif LINK_SIZE == 2
    put2ByteValue(opcodePtr, value);
#else
#   error LINK_SIZE not supported.
#endif
}

static inline int getLinkValueAllowZero(const unsigned char* opcodePtr)
{
#if LINK_SIZE == 3
    return get3ByteValue(opcodePtr);
#elif LINK_SIZE == 2
    return get2ByteValue(opcodePtr);
#else
#   error LINK_SIZE not supported.
#endif
}

#define MAX_PATTERN_SIZE 1024 * 1024 // Derived by empirical testing of compile time in PCRE and WREC.
COMPILE_ASSERT(MAX_PATTERN_SIZE < (1 << (8 * LINK_SIZE)), pcre_max_pattern_fits_in_bytecode);

static inline void putLinkValue(unsigned char* opcodePtr, int value)
{
    ASSERT(value);
    putLinkValueAllowZero(opcodePtr, value);
}

static inline int getLinkValue(const unsigned char* opcodePtr)
{
    int value = getLinkValueAllowZero(opcodePtr);
    ASSERT(value);
    return value;
}

static inline void putLinkValueAndAdvance(unsigned char*& opcodePtr, int value)
{
    putLinkValue(opcodePtr, value);
    opcodePtr += LINK_SIZE;
}

static inline void putLinkValueAllowZeroAndAdvance(unsigned char*& opcodePtr, int value)
{
    putLinkValueAllowZero(opcodePtr, value);
    opcodePtr += LINK_SIZE;
}

// FIXME: These are really more of a "compiled regexp state" than "regexp options"
enum RegExpOptions {
    UseFirstByteOptimizationOption = 0x40000000,  /* firstByte is set */
    UseRequiredByteOptimizationOption = 0x20000000,  /* reqByte is set */
    UseMultiLineFirstByteOptimizationOption = 0x10000000,  /* start after \n for multiline */
    IsAnchoredOption = 0x02000000,  /* can't use partial with this regex */
    IgnoreCaseOption = 0x00000001,
    MatchAcrossMultipleLinesOption = 0x00000002
};


#define REQ_IGNORE_CASE 0x0100    /* indicates should ignore case */
#define REQ_VARY     0x0200    /* reqByte followed non-literal item */

/* Miscellaneous definitions */


#define XCL_NOT    0x01    /* Flag: this is a negative class */
#define XCL_MAP    0x02    /* Flag: a 32-byte map is present */

#define XCL_END       0    /* Marks end of individual items */
#define XCL_SINGLE    1    /* Single item (one multibyte char) follows */
#define XCL_RANGE     2    /* A range (two multibyte chars) follows */


enum { ESC_B = 1, ESC_b, ESC_D, ESC_d, ESC_S, ESC_s, ESC_W, ESC_w, ESC_REF };


#define FOR_EACH_OPCODE(macro) \
    macro(END) \
    \
    macro(NOT_WORD_BOUNDARY) \
    macro(WORD_BOUNDARY) \
    macro(NOT_DIGIT) \
    macro(DIGIT) \
    macro(NOT_WHITESPACE) \
    macro(WHITESPACE) \
    macro(NOT_WORDCHAR) \
    macro(WORDCHAR) \
    \
    macro(NOT_NEWLINE) \
    \
    macro(CIRC) \
    macro(DOLL) \
    macro(BOL) \
    macro(EOL) \
    macro(CHAR) \
    macro(CHAR_IGNORING_CASE) \
    macro(ASCII_CHAR) \
    macro(ASCII_LETTER_IGNORING_CASE) \
    macro(NOT) \
    \
    macro(STAR) \
    macro(MINSTAR) \
    macro(PLUS) \
    macro(MINPLUS) \
    macro(QUERY) \
    macro(MINQUERY) \
    macro(UPTO) \
    macro(MINUPTO) \
    macro(EXACT) \
    \
    macro(NOTSTAR) \
    macro(NOTMINSTAR) \
    macro(NOTPLUS) \
    macro(NOTMINPLUS) \
    macro(NOTQUERY) \
    macro(NOTMINQUERY) \
    macro(NOTUPTO) \
    macro(NOTMINUPTO) \
    macro(NOTEXACT) \
    \
    macro(TYPESTAR) \
    macro(TYPEMINSTAR) \
    macro(TYPEPLUS) \
    macro(TYPEMINPLUS) \
    macro(TYPEQUERY) \
    macro(TYPEMINQUERY) \
    macro(TYPEUPTO) \
    macro(TYPEMINUPTO) \
    macro(TYPEEXACT) \
    \
    macro(CRSTAR) \
    macro(CRMINSTAR) \
    macro(CRPLUS) \
    macro(CRMINPLUS) \
    macro(CRQUERY) \
    macro(CRMINQUERY) \
    macro(CRRANGE) \
    macro(CRMINRANGE) \
    \
    macro(CLASS) \
    macro(NCLASS) \
    macro(XCLASS) \
    \
    macro(REF) \
    \
    macro(ALT) \
    macro(KET) \
    macro(KETRMAX) \
    macro(KETRMIN) \
    \
    macro(ASSERT) \
    macro(ASSERT_NOT) \
    \
    macro(BRAZERO) \
    macro(BRAMINZERO) \
    macro(BRANUMBER) \
    macro(BRA)

#define OPCODE_ENUM_VALUE(opcode) OP_##opcode,
enum { FOR_EACH_OPCODE(OPCODE_ENUM_VALUE) };




#define EXTRACT_BASIC_MAX  100

/* The code vector runs on as long as necessary after the end. */

struct JSRegExp {
    unsigned options;

    unsigned short topBracket;
    unsigned short topBackref;
    
    unsigned short firstByte;
    unsigned short reqByte;

#if REGEXP_HISTOGRAM
    size_t stringOffset;
    size_t stringLength;
#endif
};


#define jsc_pcre_utf8_table1_size 6

extern const int    jsc_pcre_utf8_table1[6];
extern const int    jsc_pcre_utf8_table2[6];
extern const int    jsc_pcre_utf8_table3[6];
extern const unsigned char jsc_pcre_utf8_table4[0x40];

extern const unsigned char jsc_pcre_default_tables[tables_length];

static inline unsigned char toLowerCase(unsigned char c)
{
    static const unsigned char* lowerCaseChars = jsc_pcre_default_tables + lcc_offset;
    return lowerCaseChars[c];
}

static inline unsigned char flipCase(unsigned char c)
{
    static const unsigned char* flippedCaseChars = jsc_pcre_default_tables + fcc_offset;
    return flippedCaseChars[c];
}

static inline unsigned char classBitmapForChar(unsigned char c)
{
    static const unsigned char* charClassBitmaps = jsc_pcre_default_tables + cbits_offset;
    return charClassBitmaps[c];
}

static inline unsigned char charTypeForChar(unsigned char c)
{
    const unsigned char* charTypeMap = jsc_pcre_default_tables + ctypes_offset;
    return charTypeMap[c];
}

static inline bool isWordChar(UChar c)
{
    return c < 128 && (charTypeForChar(c) & ctype_word);
}

static inline bool isSpaceChar(UChar c)
{
    return (c < 128 && (charTypeForChar(c) & ctype_space)) || c == 0x00A0;
}

static inline bool isNewline(UChar nl)
{
    return (nl == 0xA || nl == 0xD || nl == 0x2028 || nl == 0x2029);
}

static inline bool isBracketStartOpcode(unsigned char opcode)
{
    if (opcode >= OP_BRA)
        return true;
    switch (opcode) {
        case OP_ASSERT:
        case OP_ASSERT_NOT:
            return true;
        default:
            return false;
    }
}

static inline void advanceToEndOfBracket(const unsigned char*& opcodePtr)
{
    ASSERT(isBracketStartOpcode(*opcodePtr) || *opcodePtr == OP_ALT);
    do
        opcodePtr += getLinkValue(opcodePtr + 1);
    while (*opcodePtr == OP_ALT);
}


extern int jsc_pcre_ucp_othercase(unsigned);
extern bool jsc_pcre_xclass(int, const unsigned char*);

#endif

#endif

/* End of pcre_internal.h */
