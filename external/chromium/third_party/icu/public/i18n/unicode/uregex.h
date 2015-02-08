

#ifndef UREGEX_H
#define UREGEX_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_REGULAR_EXPRESSIONS

#include "unicode/parseerr.h"

struct URegularExpression;
typedef struct URegularExpression URegularExpression;


typedef enum URegexpFlag{

#ifndef U_HIDE_DRAFT_API 
    /** Forces normalization of pattern and strings. 
    Not implemented yet, just a placeholder, hence draft. 
    @draft ICU 2.4 */
    UREGEX_CANON_EQ         = 128,
#endif
    /**  Enable case insensitive matching.  @stable ICU 2.4 */
    UREGEX_CASE_INSENSITIVE = 2,

    /**  Allow white space and comments within patterns  @stable ICU 2.4 */
    UREGEX_COMMENTS         = 4,

    /**  If set, '.' matches line terminators,  otherwise '.' matching stops at line end.
      *  @stable ICU 2.4 */
    UREGEX_DOTALL           = 32,
    
    /**  If set, treat the entire pattern as a literal string.  
      *  Metacharacters or escape sequences in the input sequence will be given 
      *  no special meaning.
      *
      *  The flags CASE_INSENSITIVE and UNICODE_CASE retain their impact
      *  on matching when used in conjunction with this flag.
      *  The other flags become superfluous.
      *  TODO:  say which escapes are still handled; anything Java does
      *         early (\\u) we should still do.
      * @stable ICU 4.0
      */
    UREGEX_LITERAL = 16,

    /**   Control behavior of "$" and "^"
      *    If set, recognize line terminators within string,
      *    otherwise, match only at start and end of input string.
      *   @stable ICU 2.4 */
    UREGEX_MULTILINE        = 8,
    
    /**   Unix-only line endings.
      *   When this mode is enabled, only \\u000a is recognized as a line ending
      *    in the behavior of ., ^, and $.
      *   @stable ICU 4.0
      */
    UREGEX_UNIX_LINES = 1,

    /**  Unicode word boundaries.
      *     If set, \b uses the Unicode TR 29 definition of word boundaries.
      *     Warning: Unicode word boundaries are quite different from
      *     traditional regular expression word boundaries.  See
      *     http://unicode.org/reports/tr29/#Word_Boundaries
      *     @stable ICU 2.8
      */
    UREGEX_UWORD            = 256,

     /**  Error on Unrecognized backslash escapes.
       *     If set, fail with an error on patterns that contain
       *     backslash-escaped ASCII letters without a known specail
       *     meaning.  If this flag is not set, these
       *     escaped letters represent themselves.
       *     @stable ICU 4.0
       */
     UREGEX_ERROR_ON_UNKNOWN_ESCAPES = 512

}  URegexpFlag;

U_STABLE URegularExpression * U_EXPORT2
uregex_open( const  UChar          *pattern,
                    int32_t         patternLength,
                    uint32_t        flags,
                    UParseError    *pe,
                    UErrorCode     *status);

#if !UCONFIG_NO_CONVERSION
U_STABLE URegularExpression * U_EXPORT2
uregex_openC( const char           *pattern,
                    uint32_t        flags,
                    UParseError    *pe,
                    UErrorCode     *status);
#endif



U_STABLE void U_EXPORT2 
uregex_close(URegularExpression *regexp);

U_STABLE URegularExpression * U_EXPORT2 
uregex_clone(const URegularExpression *regexp, UErrorCode *status);

U_STABLE const UChar * U_EXPORT2 
uregex_pattern(const  URegularExpression   *regexp,
                         int32_t           *patLength,
                         UErrorCode        *status);


U_STABLE int32_t U_EXPORT2 
uregex_flags(const  URegularExpression   *regexp,
                    UErrorCode           *status);


U_STABLE void U_EXPORT2 
uregex_setText(URegularExpression *regexp,
               const UChar        *text,
               int32_t             textLength,
               UErrorCode         *status);

U_STABLE const UChar * U_EXPORT2 
uregex_getText(URegularExpression *regexp,
               int32_t            *textLength,
               UErrorCode         *status);

U_STABLE UBool U_EXPORT2 
uregex_matches(URegularExpression *regexp,
                int32_t            startIndex,
                UErrorCode        *status);

U_STABLE UBool U_EXPORT2 
uregex_lookingAt(URegularExpression *regexp,
                 int32_t             startIndex,
                 UErrorCode         *status);

U_STABLE UBool U_EXPORT2 
uregex_find(URegularExpression *regexp,
            int32_t             startIndex, 
            UErrorCode         *status);

U_STABLE UBool U_EXPORT2 
uregex_findNext(URegularExpression *regexp,
                UErrorCode         *status);

U_STABLE int32_t U_EXPORT2 
uregex_groupCount(URegularExpression *regexp,
                  UErrorCode         *status);

U_STABLE int32_t U_EXPORT2 
uregex_group(URegularExpression *regexp,
             int32_t             groupNum,
             UChar              *dest,
             int32_t             destCapacity,
             UErrorCode          *status);


U_STABLE int32_t U_EXPORT2 
uregex_start(URegularExpression *regexp,
             int32_t             groupNum,
             UErrorCode          *status);

U_STABLE int32_t U_EXPORT2 
uregex_end(URegularExpression   *regexp,
           int32_t               groupNum,
           UErrorCode           *status);

U_STABLE void U_EXPORT2 
uregex_reset(URegularExpression    *regexp,
             int32_t               index,
             UErrorCode            *status);
             
             
U_DRAFT void U_EXPORT2
uregex_setRegion(URegularExpression   *regexp,
                 int32_t               regionStart,
                 int32_t               regionLimit,
                 UErrorCode           *status);

U_DRAFT int32_t U_EXPORT2
uregex_regionStart(const  URegularExpression   *regexp,
                          UErrorCode           *status);



U_DRAFT int32_t U_EXPORT2
uregex_regionEnd(const  URegularExpression   *regexp,
                        UErrorCode           *status);

U_DRAFT UBool U_EXPORT2
uregex_hasTransparentBounds(const  URegularExpression   *regexp,
                                   UErrorCode           *status);


U_DRAFT void U_EXPORT2  
uregex_useTransparentBounds(URegularExpression   *regexp, 
                            UBool                b,
                            UErrorCode           *status);


U_DRAFT UBool U_EXPORT2
uregex_hasAnchoringBounds(const  URegularExpression   *regexp,
                                 UErrorCode           *status);


U_DRAFT void U_EXPORT2
uregex_useAnchoringBounds(URegularExpression   *regexp,
                          UBool                 b,
                          UErrorCode           *status);

U_DRAFT UBool U_EXPORT2
uregex_hitEnd(const  URegularExpression   *regexp,
                     UErrorCode           *status);

U_DRAFT UBool U_EXPORT2   
uregex_requireEnd(const  URegularExpression   *regexp,
                         UErrorCode           *status);





U_STABLE int32_t U_EXPORT2 
uregex_replaceAll(URegularExpression    *regexp,
                  const UChar           *replacementText,
                  int32_t                replacementLength,
                  UChar                 *destBuf,
                  int32_t                destCapacity,
                  UErrorCode            *status);


U_STABLE int32_t U_EXPORT2 
uregex_replaceFirst(URegularExpression  *regexp,
                    const UChar         *replacementText,
                    int32_t              replacementLength,
                    UChar               *destBuf,
                    int32_t              destCapacity,
                    UErrorCode          *status);


U_STABLE int32_t U_EXPORT2 
uregex_appendReplacement(URegularExpression    *regexp,
                  const UChar           *replacementText,
                  int32_t                replacementLength,
                  UChar                **destBuf,
                  int32_t               *destCapacity,
                  UErrorCode            *status);


U_STABLE int32_t U_EXPORT2 
uregex_appendTail(URegularExpression    *regexp,
                  UChar                **destBuf,
                  int32_t               *destCapacity,
                  UErrorCode            *status);




 /**
   * Split a string into fields.  Somewhat like split() from Perl.
   *  The pattern matches identify delimiters that separate the input
   *  into fields.  The input data between the matches becomes the
   *  fields themselves.
   * <p>
   *  Each of the fields is copied from the input string to the destination
   *  buffer, and the NUL terminated.  The position of each field within
   *  the destination buffer is returned in the destFields array.
   *
   *  Note:  another choice for the design of this function would be to not
   *         copy the resulting fields at all, but to return indexes and
   *         lengths within the source text.  
   *           Advantages would be
   *             o  Faster.  No Copying.
   *             o  Nothing extra needed when field data may contain embedded NUL chars.
   *             o  Less memory needed if working on large data.
   *           Disadvantages
   *             o  Less consistent with C++ split, which copies into an
   *                array of UnicodeStrings.
   *             o  No NUL termination, extracted fields would be less convenient
   *                to use in most cases.
   *             o  Possible problems in the future, when support Unicode Normalization
   *                could cause the fields to not correspond exactly to
   *                a range of the source text.
   * 
   *    @param   regexp      The compiled regular expression.
   *    @param   destBuf     A (UChar *) buffer to receive the fields that
   *                         are extracted from the input string. These
   *                         field pointers will refer to positions within the
   *                         destination buffer supplied by the caller.  Any
   *                         extra positions within the destFields array will be
   *                         set to NULL.
   *    @param   destCapacity The capacity of the destBuf.
   *    @param   requiredCapacity  The actual capacity required of the destBuf.
   *                         If destCapacity is too small, requiredCapacity will return 
   *                         the total capacity required to hold all of the output, and
   *                         a U_BUFFER_OVERFLOW_ERROR will be returned.
   *    @param   destFields  An array to be filled with the position of each
   *                         of the extracted fields within destBuf.
   *    @param   destFieldsCapacity  The number of elements in the destFields array.
   *                If the number of fields found is less than destFieldsCapacity,
   *                the extra destFields elements are set to zero.
   *                If destFieldsCapacity is too small, the trailing part of the
   *                input, including any field delimiters, is treated as if it
   *                were the last field - it is copied to the destBuf, and
   *                its position is in the destBuf is stored in the last element
   *                of destFields.  This behavior mimics that of Perl.  It is not
   *                an error condition, and no error status is returned when all destField
   *                positions are used.
   * @param status  A reference to a UErrorCode to receive any errors.
   * @return        The number of fields into which the input string was split.
   * @stable ICU 3.0
   */
U_STABLE int32_t U_EXPORT2 
uregex_split(   URegularExpression      *regexp,
                  UChar                 *destBuf,
                  int32_t                destCapacity,
                  int32_t               *requiredCapacity,
                  UChar                 *destFields[],
                  int32_t                destFieldsCapacity,
                  UErrorCode            *status);




U_DRAFT void U_EXPORT2
uregex_setTimeLimit(URegularExpression      *regexp,
                    int32_t                  limit,
                    UErrorCode              *status);

U_DRAFT int32_t U_EXPORT2
uregex_getTimeLimit(const URegularExpression      *regexp,
                          UErrorCode              *status);

U_DRAFT void U_EXPORT2
uregex_setStackLimit(URegularExpression      *regexp,
                     int32_t                  limit,
                     UErrorCode              *status);

U_DRAFT int32_t U_EXPORT2
uregex_getStackLimit(const URegularExpression      *regexp,
                           UErrorCode              *status);


U_CDECL_BEGIN
typedef UBool U_CALLCONV URegexMatchCallback (
                   const void *context,
                   int32_t     steps);
U_CDECL_END

U_DRAFT void U_EXPORT2
uregex_setMatchCallback(URegularExpression      *regexp,
                        URegexMatchCallback     *callback,
                        const void              *context,
                        UErrorCode              *status);


U_DRAFT void U_EXPORT2
uregex_getMatchCallback(const URegularExpression    *regexp,
                        URegexMatchCallback        **callback,
                        const void                 **context,
                        UErrorCode                  *status);



#endif   /*  !UCONFIG_NO_REGULAR_EXPRESSIONS  */
#endif   /*  UREGEX_H  */
