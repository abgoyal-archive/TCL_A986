

#ifndef USTRING_H
#define USTRING_H

#include "unicode/utypes.h"
#include "unicode/putil.h"
#include "unicode/uiter.h"

/** Simple declaration for u_strToTitle() to avoid including unicode/ubrk.h. @stable ICU 2.1*/
#ifndef UBRK_TYPEDEF_UBREAK_ITERATOR
#   define UBRK_TYPEDEF_UBREAK_ITERATOR
    typedef void UBreakIterator;
#endif


U_STABLE int32_t U_EXPORT2
u_strlen(const UChar *s);

U_STABLE int32_t U_EXPORT2
u_countChar32(const UChar *s, int32_t length);

U_STABLE UBool U_EXPORT2
u_strHasMoreChar32Than(const UChar *s, int32_t length, int32_t number);

U_STABLE UChar* U_EXPORT2
u_strcat(UChar     *dst, 
    const UChar     *src);

U_STABLE UChar* U_EXPORT2
u_strncat(UChar     *dst, 
     const UChar     *src, 
     int32_t     n);

U_STABLE UChar * U_EXPORT2
u_strstr(const UChar *s, const UChar *substring);

U_STABLE UChar * U_EXPORT2
u_strFindFirst(const UChar *s, int32_t length, const UChar *substring, int32_t subLength);

U_STABLE UChar * U_EXPORT2
u_strchr(const UChar *s, UChar c);

U_STABLE UChar * U_EXPORT2
u_strchr32(const UChar *s, UChar32 c);

U_STABLE UChar * U_EXPORT2
u_strrstr(const UChar *s, const UChar *substring);

U_STABLE UChar * U_EXPORT2
u_strFindLast(const UChar *s, int32_t length, const UChar *substring, int32_t subLength);

U_STABLE UChar * U_EXPORT2
u_strrchr(const UChar *s, UChar c);

U_STABLE UChar * U_EXPORT2
u_strrchr32(const UChar *s, UChar32 c);

U_STABLE UChar * U_EXPORT2
u_strpbrk(const UChar *string, const UChar *matchSet);

U_STABLE int32_t U_EXPORT2
u_strcspn(const UChar *string, const UChar *matchSet);

U_STABLE int32_t U_EXPORT2
u_strspn(const UChar *string, const UChar *matchSet);

U_STABLE UChar * U_EXPORT2
u_strtok_r(UChar    *src, 
     const UChar    *delim,
           UChar   **saveState);

U_STABLE int32_t  U_EXPORT2
u_strcmp(const UChar     *s1, 
         const UChar     *s2);

U_STABLE int32_t U_EXPORT2
u_strcmpCodePointOrder(const UChar *s1, const UChar *s2);

U_STABLE int32_t U_EXPORT2
u_strCompare(const UChar *s1, int32_t length1,
             const UChar *s2, int32_t length2,
             UBool codePointOrder);

U_STABLE int32_t U_EXPORT2
u_strCompareIter(UCharIterator *iter1, UCharIterator *iter2, UBool codePointOrder);

#ifndef U_COMPARE_CODE_POINT_ORDER
/* see also unistr.h and unorm.h */
#define U_COMPARE_CODE_POINT_ORDER  0x8000
#endif

U_STABLE int32_t U_EXPORT2
u_strCaseCompare(const UChar *s1, int32_t length1,
                 const UChar *s2, int32_t length2,
                 uint32_t options,
                 UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
u_strncmp(const UChar     *ucs1, 
     const UChar     *ucs2, 
     int32_t     n);

U_STABLE int32_t U_EXPORT2
u_strncmpCodePointOrder(const UChar *s1, const UChar *s2, int32_t n);

U_STABLE int32_t U_EXPORT2
u_strcasecmp(const UChar *s1, const UChar *s2, uint32_t options);

U_STABLE int32_t U_EXPORT2
u_strncasecmp(const UChar *s1, const UChar *s2, int32_t n, uint32_t options);

U_STABLE int32_t U_EXPORT2
u_memcasecmp(const UChar *s1, const UChar *s2, int32_t length, uint32_t options);

U_STABLE UChar* U_EXPORT2
u_strcpy(UChar     *dst, 
    const UChar     *src);

U_STABLE UChar* U_EXPORT2
u_strncpy(UChar     *dst, 
     const UChar     *src, 
     int32_t     n);

#if !UCONFIG_NO_CONVERSION

U_STABLE UChar* U_EXPORT2 u_uastrcpy(UChar *dst,
               const char *src );

U_STABLE UChar* U_EXPORT2 u_uastrncpy(UChar *dst,
            const char *src,
            int32_t n);

U_STABLE char* U_EXPORT2 u_austrcpy(char *dst,
            const UChar *src );

U_STABLE char* U_EXPORT2 u_austrncpy(char *dst,
            const UChar *src,
            int32_t n );

#endif

U_STABLE UChar* U_EXPORT2
u_memcpy(UChar *dest, const UChar *src, int32_t count);

U_STABLE UChar* U_EXPORT2
u_memmove(UChar *dest, const UChar *src, int32_t count);

U_STABLE UChar* U_EXPORT2
u_memset(UChar *dest, UChar c, int32_t count);

U_STABLE int32_t U_EXPORT2
u_memcmp(const UChar *buf1, const UChar *buf2, int32_t count);

U_STABLE int32_t U_EXPORT2
u_memcmpCodePointOrder(const UChar *s1, const UChar *s2, int32_t count);

U_STABLE UChar* U_EXPORT2
u_memchr(const UChar *s, UChar c, int32_t count);

U_STABLE UChar* U_EXPORT2
u_memchr32(const UChar *s, UChar32 c, int32_t count);

U_STABLE UChar* U_EXPORT2
u_memrchr(const UChar *s, UChar c, int32_t count);

U_STABLE UChar* U_EXPORT2
u_memrchr32(const UChar *s, UChar32 c, int32_t count);

#if U_SIZEOF_WCHAR_T==U_SIZEOF_UCHAR && U_CHARSET_FAMILY==U_ASCII_FAMILY
#   define U_STRING_DECL(var, cs, length) static const wchar_t var[(length)+1]={ L ## cs }
    /**@stable ICU 2.0 */
#   define U_STRING_INIT(var, cs, length)
#elif U_SIZEOF_UCHAR==1 && U_CHARSET_FAMILY==U_ASCII_FAMILY
#   define U_STRING_DECL(var, cs, length) static const UChar var[(length)+1]={ (const UChar *)cs }
    /**@stable ICU 2.0 */
#   define U_STRING_INIT(var, cs, length)
#else
#   define U_STRING_DECL(var, cs, length) static UChar var[(length)+1]
    /**@stable ICU 2.0 */
#   define U_STRING_INIT(var, cs, length) u_charsToUChars(cs, var, length+1)
#endif

U_STABLE int32_t U_EXPORT2
u_unescape(const char *src,
           UChar *dest, int32_t destCapacity);

U_CDECL_BEGIN
typedef UChar (U_CALLCONV *UNESCAPE_CHAR_AT)(int32_t offset, void *context);
U_CDECL_END

U_STABLE UChar32 U_EXPORT2
u_unescapeAt(UNESCAPE_CHAR_AT charAt,
             int32_t *offset,
             int32_t length,
             void *context);

U_STABLE int32_t U_EXPORT2
u_strToUpper(UChar *dest, int32_t destCapacity,
             const UChar *src, int32_t srcLength,
             const char *locale,
             UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
u_strToLower(UChar *dest, int32_t destCapacity,
             const UChar *src, int32_t srcLength,
             const char *locale,
             UErrorCode *pErrorCode);

#if !UCONFIG_NO_BREAK_ITERATION

U_STABLE int32_t U_EXPORT2
u_strToTitle(UChar *dest, int32_t destCapacity,
             const UChar *src, int32_t srcLength,
             UBreakIterator *titleIter,
             const char *locale,
             UErrorCode *pErrorCode);

#endif

U_STABLE int32_t U_EXPORT2
u_strFoldCase(UChar *dest, int32_t destCapacity,
              const UChar *src, int32_t srcLength,
              uint32_t options,
              UErrorCode *pErrorCode);

U_STABLE wchar_t* U_EXPORT2
u_strToWCS(wchar_t *dest, 
           int32_t destCapacity,
           int32_t *pDestLength,
           const UChar *src, 
           int32_t srcLength,
           UErrorCode *pErrorCode);
U_STABLE UChar* U_EXPORT2
u_strFromWCS(UChar   *dest,
             int32_t destCapacity, 
             int32_t *pDestLength,
             const wchar_t *src,
             int32_t srcLength,
             UErrorCode *pErrorCode);
U_STABLE char* U_EXPORT2 
u_strToUTF8(char *dest,           
            int32_t destCapacity,
            int32_t *pDestLength,
            const UChar *src, 
            int32_t srcLength,
            UErrorCode *pErrorCode);

U_STABLE UChar* U_EXPORT2
u_strFromUTF8(UChar *dest,             
              int32_t destCapacity,
              int32_t *pDestLength,
              const char *src, 
              int32_t srcLength,
              UErrorCode *pErrorCode);

U_STABLE UChar32* U_EXPORT2 
u_strToUTF32(UChar32 *dest, 
             int32_t  destCapacity,
             int32_t  *pDestLength,
             const UChar *src, 
             int32_t  srcLength,
             UErrorCode *pErrorCode);

U_STABLE UChar* U_EXPORT2 
u_strFromUTF32(UChar   *dest,
               int32_t destCapacity, 
               int32_t *pDestLength,
               const UChar32 *src,
               int32_t srcLength,
               UErrorCode *pErrorCode);

#endif
