
#ifndef UMSG_H
#define UMSG_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/uloc.h"
#include "unicode/parseerr.h"
#include <stdarg.h>

U_STABLE int32_t U_EXPORT2 
u_formatMessage(const char  *locale,
                 const UChar *pattern,
                int32_t     patternLength,
                UChar       *result,
                int32_t     resultLength,
                UErrorCode  *status,
                ...);

U_STABLE int32_t U_EXPORT2 
u_vformatMessage(   const char  *locale,
                    const UChar *pattern,
                    int32_t     patternLength,
                    UChar       *result,
                    int32_t     resultLength,
                    va_list     ap,
                    UErrorCode  *status);

U_STABLE void U_EXPORT2 
u_parseMessage( const char   *locale,
                const UChar  *pattern,
                int32_t      patternLength,
                const UChar  *source,
                int32_t      sourceLength,
                UErrorCode   *status,
                ...);

U_STABLE void U_EXPORT2 
u_vparseMessage(const char  *locale,
                const UChar *pattern,
                int32_t     patternLength,
                const UChar *source,
                int32_t     sourceLength,
                va_list     ap,
                UErrorCode  *status);

U_STABLE int32_t U_EXPORT2 
u_formatMessageWithError(   const char    *locale,
                            const UChar   *pattern,
                            int32_t       patternLength,
                            UChar         *result,
                            int32_t       resultLength,
                            UParseError   *parseError,
                            UErrorCode    *status,
                            ...);

U_STABLE int32_t U_EXPORT2 
u_vformatMessageWithError(  const char   *locale,
                            const UChar  *pattern,
                            int32_t      patternLength,
                            UChar        *result,
                            int32_t      resultLength,
                            UParseError* parseError,
                            va_list      ap,
                            UErrorCode   *status);

U_STABLE void U_EXPORT2 
u_parseMessageWithError(const char  *locale,
                        const UChar *pattern,
                        int32_t     patternLength,
                        const UChar *source,
                        int32_t     sourceLength,
                        UParseError *parseError,
                        UErrorCode  *status,
                        ...);

U_STABLE void U_EXPORT2 
u_vparseMessageWithError(const char  *locale,
                         const UChar *pattern,
                         int32_t     patternLength,
                         const UChar *source,
                         int32_t     sourceLength,
                         va_list     ap,
                         UParseError *parseError,
                         UErrorCode* status);

/*----------------------- New experimental API --------------------------- */
typedef void* UMessageFormat;


U_STABLE UMessageFormat* U_EXPORT2 
umsg_open(  const UChar     *pattern,
            int32_t         patternLength,
            const  char     *locale,
            UParseError     *parseError,
            UErrorCode      *status);

U_STABLE void U_EXPORT2 
umsg_close(UMessageFormat* format);

U_STABLE UMessageFormat U_EXPORT2 
umsg_clone(const UMessageFormat *fmt,
           UErrorCode *status);

U_STABLE void  U_EXPORT2 
umsg_setLocale(UMessageFormat *fmt,
               const char* locale);

U_STABLE const char*  U_EXPORT2 
umsg_getLocale(const UMessageFormat *fmt);

U_STABLE void  U_EXPORT2 
umsg_applyPattern( UMessageFormat *fmt,
                   const UChar* pattern,
                   int32_t patternLength,
                   UParseError* parseError,
                   UErrorCode* status);

U_STABLE int32_t  U_EXPORT2 
umsg_toPattern(const UMessageFormat *fmt,
               UChar* result, 
               int32_t resultLength,
               UErrorCode* status);

U_STABLE int32_t U_EXPORT2 
umsg_format(    const UMessageFormat *fmt,
                UChar          *result,
                int32_t        resultLength,
                UErrorCode     *status,
                ...);

U_STABLE int32_t U_EXPORT2 
umsg_vformat(   const UMessageFormat *fmt,
                UChar          *result,
                int32_t        resultLength,
                va_list        ap,
                UErrorCode     *status);

U_STABLE void U_EXPORT2 
umsg_parse( const UMessageFormat *fmt,
            const UChar    *source,
            int32_t        sourceLength,
            int32_t        *count,
            UErrorCode     *status,
            ...);

U_STABLE void U_EXPORT2 
umsg_vparse(const UMessageFormat *fmt,
            const UChar    *source,
            int32_t        sourceLength,
            int32_t        *count,
            va_list        ap,
            UErrorCode     *status);


U_STABLE int32_t U_EXPORT2 
umsg_autoQuoteApostrophe(const UChar* pattern, 
                         int32_t patternLength,
                         UChar* dest,
                         int32_t destCapacity,
                         UErrorCode* ec);

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
