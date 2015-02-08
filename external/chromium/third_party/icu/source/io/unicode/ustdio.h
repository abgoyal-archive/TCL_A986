
#ifndef USTDIO_H
#define USTDIO_H

#include <stdio.h>
#include <stdarg.h>

#include "unicode/utypes.h"
#include "unicode/ucnv.h"
#include "unicode/utrans.h"




#define U_EOF 0xFFFF

/** Forward declaration of a Unicode-aware file @stable 3.0 */
typedef struct UFILE UFILE;

typedef enum { 
   U_READ = 1,
   U_WRITE = 2, 
   U_READWRITE =3  /* == (U_READ | U_WRITE) */ 
} UFileDirection;

U_DRAFT UFILE* U_EXPORT2
u_fopen(const char    *filename,
    const char    *perm,
    const char    *locale,
    const char    *codepage);

U_DRAFT UFILE* U_EXPORT2
u_finit(FILE        *f,
    const char    *locale,
    const char    *codepage);

U_DRAFT UFILE* U_EXPORT2
u_fstropen(UChar      *stringBuf,
           int32_t     capacity,
           const char *locale);

U_DRAFT void U_EXPORT2
u_fclose(UFILE *file);

U_DRAFT UBool U_EXPORT2
u_feof(UFILE  *f);

U_DRAFT void U_EXPORT2
u_fflush(UFILE *file);

U_DRAFT void
u_frewind(UFILE *file);

U_DRAFT FILE* U_EXPORT2
u_fgetfile(UFILE *f);

#if !UCONFIG_NO_FORMATTING

U_DRAFT const char* U_EXPORT2
u_fgetlocale(UFILE *file);

U_DRAFT int32_t U_EXPORT2
u_fsetlocale(UFILE      *file,
             const char *locale);

#endif

U_DRAFT const char* U_EXPORT2
u_fgetcodepage(UFILE *file);

U_DRAFT int32_t U_EXPORT2
u_fsetcodepage(const char   *codepage,
               UFILE        *file);


U_DRAFT UConverter* U_EXPORT2 u_fgetConverter(UFILE *f);

#if !UCONFIG_NO_FORMATTING

/* Output functions */

U_DRAFT int32_t U_EXPORT2
u_fprintf(UFILE         *f,
          const char    *patternSpecification,
          ... );

U_DRAFT int32_t U_EXPORT2
u_vfprintf(UFILE        *f,
           const char   *patternSpecification,
           va_list      ap);

U_DRAFT int32_t U_EXPORT2
u_fprintf_u(UFILE       *f,
            const UChar *patternSpecification,
            ... );

U_DRAFT int32_t U_EXPORT2
u_vfprintf_u(UFILE      *f,
            const UChar *patternSpecification,
            va_list     ap);
#endif
U_DRAFT int32_t U_EXPORT2
u_fputs(const UChar *s,
        UFILE       *f);

U_DRAFT UChar32 U_EXPORT2
u_fputc(UChar32  uc,
        UFILE  *f);

U_DRAFT int32_t U_EXPORT2
u_file_write(const UChar    *ustring, 
             int32_t        count, 
             UFILE          *f);


/* Input functions */
#if !UCONFIG_NO_FORMATTING

U_DRAFT int32_t U_EXPORT2
u_fscanf(UFILE      *f,
         const char *patternSpecification,
         ... );

U_DRAFT int32_t U_EXPORT2
u_vfscanf(UFILE         *f,
          const char    *patternSpecification,
          va_list        ap);

U_DRAFT int32_t U_EXPORT2
u_fscanf_u(UFILE        *f,
           const UChar  *patternSpecification,
           ... );

U_DRAFT int32_t U_EXPORT2
u_vfscanf_u(UFILE       *f,
            const UChar *patternSpecification,
            va_list      ap);
#endif

U_DRAFT UChar* U_EXPORT2
u_fgets(UChar  *s,
        int32_t n,
        UFILE  *f);

U_DRAFT UChar U_EXPORT2
u_fgetc(UFILE   *f);

U_DRAFT UChar32 U_EXPORT2
u_fgetcx(UFILE  *f);

U_DRAFT UChar32 U_EXPORT2
u_fungetc(UChar32   c,
      UFILE        *f);

U_DRAFT int32_t U_EXPORT2
u_file_read(UChar        *chars, 
        int32_t        count, 
        UFILE         *f);

#if !UCONFIG_NO_TRANSLITERATION

U_DRAFT UTransliterator* U_EXPORT2
u_fsettransliterator(UFILE *file, UFileDirection direction,
                     UTransliterator *adopt, UErrorCode *status);

#endif


/* Output string functions */
#if !UCONFIG_NO_FORMATTING


U_DRAFT int32_t U_EXPORT2
u_sprintf(UChar       *buffer,
        const char    *patternSpecification,
        ... );

U_DRAFT int32_t U_EXPORT2
u_snprintf(UChar      *buffer,
        int32_t       count,
        const char    *patternSpecification,
        ... );

U_DRAFT int32_t U_EXPORT2
u_vsprintf(UChar      *buffer,
        const char    *patternSpecification,
        va_list        ap);

U_DRAFT int32_t U_EXPORT2
u_vsnprintf(UChar     *buffer,
        int32_t       count,
        const char    *patternSpecification,
        va_list        ap);

U_DRAFT int32_t U_EXPORT2
u_sprintf_u(UChar      *buffer,
        const UChar    *patternSpecification,
        ... );

U_DRAFT int32_t U_EXPORT2
u_snprintf_u(UChar     *buffer,
        int32_t        count,
        const UChar    *patternSpecification,
        ... );

U_DRAFT int32_t U_EXPORT2
u_vsprintf_u(UChar     *buffer,
        const UChar    *patternSpecification,
        va_list        ap);

U_DRAFT int32_t U_EXPORT2
u_vsnprintf_u(UChar *buffer,
        int32_t         count,
        const UChar     *patternSpecification,
        va_list         ap);

/* Input string functions */

U_DRAFT int32_t U_EXPORT2
u_sscanf(const UChar   *buffer,
        const char     *patternSpecification,
        ... );

U_DRAFT int32_t U_EXPORT2
u_vsscanf(const UChar  *buffer,
        const char     *patternSpecification,
        va_list        ap);

U_DRAFT int32_t U_EXPORT2
u_sscanf_u(const UChar  *buffer,
        const UChar     *patternSpecification,
        ... );

U_DRAFT int32_t U_EXPORT2
u_vsscanf_u(const UChar *buffer,
        const UChar     *patternSpecification,
        va_list         ap);

#endif
#endif


