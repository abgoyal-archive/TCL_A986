
#ifndef UFMT_CMN_H
#define UFMT_CMN_H

#include "unicode/utypes.h"

#define UFMT_DEFAULT_BUFFER_SIZE 128
#define MAX_UCHAR_BUFFER_SIZE(buffer) (sizeof(buffer)/(UTF_MAX_CHAR_LENGTH*sizeof(UChar)))
#define MAX_UCHAR_BUFFER_NEEDED(strLen) ((strLen+1)*UTF_MAX_CHAR_LENGTH*sizeof(UChar))

typedef enum ufmt_type_info {
    ufmt_empty = 0,
    ufmt_simple_percent, /* %% do nothing */
    ufmt_count,      /* special flag for count */
    ufmt_int,        /* int */
    ufmt_char,       /* int, cast to char */
    ufmt_string,     /* char* */
    ufmt_pointer,    /* void* */
    ufmt_float,      /* float */
    ufmt_double,     /* double */
    ufmt_uchar,      /* int, cast to UChar */
    ufmt_ustring     /* UChar* */
    /*ufmt_wchar,*/      /* wchar_t */
    /*ufmt_wstring,*/    /* wchar_t* */
    /*ufmt_date,*/       /* Date */
    /*ufmt_last*/
} ufmt_type_info;

typedef union ufmt_args {
    int64_t int64Value;    /* int, UChar */
    float   floatValue;    /* float */
    double  doubleValue;   /* double */
    void    *ptrValue;     /* any pointer - void*, char*, wchar_t*, UChar* */
    /*wchar_t wcharValue;*/    /* wchar_t */    /* TODO: Should wchar_t be used? */
    /*UDate dateValue;*/     /* Date */
} ufmt_args;

#define ufmt_min(a,b) ((a) < (b) ? (a) : (b))

int
ufmt_digitvalue(UChar c);

UBool
ufmt_isdigit(UChar     c,
         int32_t     radix);

void 
ufmt_64tou(UChar     *buffer, 
      int32_t     *len,
      uint64_t     value, 
      uint8_t     radix,
      UBool    uselower,
      int32_t    minDigits);

void 
ufmt_ptou(UChar    *buffer, 
          int32_t   *len,
          void      *value, 
          UBool     uselower);

int64_t
ufmt_uto64(const UChar     *buffer, 
      int32_t     *len,
      int8_t     radix);

void *
ufmt_utop(const UChar     *buffer,
      int32_t     *len);

UChar*
ufmt_defaultCPToUnicode(const char *s, int32_t sSize,
                        UChar *target, int32_t tSize);



#endif

