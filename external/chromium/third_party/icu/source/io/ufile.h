
#ifndef UFILE_H
#define UFILE_H

#include "unicode/utypes.h"
#include "unicode/ucnv.h"
#include "unicode/utrans.h"
#include "locbund.h"

/* The buffer size for fromUnicode calls */
#define UFILE_CHARBUFFER_SIZE 1024

/* The buffer size for toUnicode calls */
#define UFILE_UCHARBUFFER_SIZE 1024

/* A UFILE */

#if !UCONFIG_NO_TRANSLITERATION

typedef struct {
    UChar  *buffer;             /* Beginning of buffer */
    int32_t capacity;           /* Capacity of buffer */
    int32_t pos;                /* Beginning of untranslitted data */
    int32_t length;             /* Length *from beginning of buffer* of untranslitted data */
    UTransliterator *translit;
} UFILETranslitBuffer;

#endif

typedef struct u_localized_string {
    UChar       *fPos;          /* current pos in fUCBuffer */
    const UChar *fLimit;        /* data limit in fUCBuffer */
    UChar       *fBuffer;       /* Place to write the string */

#if !UCONFIG_NO_FORMATTING
    ULocaleBundle  fBundle; /* formatters */
#endif
} u_localized_string;

struct UFILE {
#if !UCONFIG_NO_TRANSLITERATION
    UFILETranslitBuffer *fTranslit;
#endif

    FILE        *fFile;         /* the actual filesystem interface */

    UConverter  *fConverter;    /* for codeset conversion */

    u_localized_string str;     /* struct to handle strings for number formatting */

    UChar       fUCBuffer[UFILE_UCHARBUFFER_SIZE];/* buffer used for toUnicode */

    UBool       fOwnFile;       /* TRUE if fFile should be closed */

    int32_t     fFileno;        /* File number. Useful to determine if it's stdin. */
};

U_CFUNC int32_t U_EXPORT2
u_file_write_flush( const UChar     *chars, 
        int32_t     count, 
        UFILE       *f,
        UBool       flushIO,
        UBool       flushTranslit);

void
ufile_fill_uchar_buffer(UFILE *f);

U_CFUNC UBool U_EXPORT2
ufile_getch(UFILE *f, UChar *ch);

U_CFUNC UBool U_EXPORT2
ufile_getch32(UFILE *f, UChar32 *ch);

void 
ufile_close_translit(UFILE *f);

void 
ufile_flush_translit(UFILE *f);


#endif
