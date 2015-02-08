
#include "unicode/ucnv.h"
#include "filestrm.h"

#if !UCONFIG_NO_CONVERSION

#ifndef UCBUF_H
#define UCBUF_H 1

typedef struct UCHARBUF UCHARBUF;
#define U_EOF 0xFFFFFFFF
#define U_ERR 0xFFFFFFFE

typedef struct ULine ULine;

struct  ULine {
    UChar     *name;
    int32_t   len;
};

U_CAPI UCHARBUF* U_EXPORT2
ucbuf_open(const char* fileName,const char** codepage,UBool showWarning, UBool buffered, UErrorCode* err);

U_CAPI int32_t U_EXPORT2
ucbuf_getc(UCHARBUF* buf,UErrorCode* err);

U_CAPI int32_t U_EXPORT2
ucbuf_getc32(UCHARBUF* buf,UErrorCode* err);

U_CAPI int32_t U_EXPORT2
ucbuf_getcx32(UCHARBUF* buf,UErrorCode* err);

U_CAPI const UChar* U_EXPORT2
ucbuf_readline(UCHARBUF* buf,int32_t* len, UErrorCode* err);


U_CAPI void U_EXPORT2
ucbuf_rewind(UCHARBUF* buf,UErrorCode* err);

U_CAPI const UChar* U_EXPORT2
ucbuf_getBuffer(UCHARBUF* buf,int32_t* len,UErrorCode* err);

U_CAPI void U_EXPORT2
ucbuf_close(UCHARBUF* buf);

U_CAPI void U_EXPORT2
ucbuf_ungetc(int32_t ungetChar,UCHARBUF* buf);


U_CAPI FileStream * U_EXPORT2
ucbuf_autodetect(const char* fileName, const char** cp,UConverter** conv,
int32_t* signatureLength, UErrorCode* status);


U_CAPI UBool U_EXPORT2
ucbuf_autodetect_fs(FileStream* in, const char** cp, UConverter** conv, int32_t* signatureLength, UErrorCode* status);

U_CAPI int32_t U_EXPORT2
ucbuf_size(UCHARBUF* buf);

U_CAPI const char* U_EXPORT2
ucbuf_resolveFileName(const char* inputDir, const char* fileName, char* target, int32_t* len, UErrorCode* status);

#endif
#endif

