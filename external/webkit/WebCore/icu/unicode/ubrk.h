

#ifndef UBRK_H
#define UBRK_H

#include "unicode/utypes.h"
#include "unicode/uloc.h"

#ifndef UBRK_TYPEDEF_UBREAK_ITERATOR
#   define UBRK_TYPEDEF_UBREAK_ITERATOR
    /**
     *  Opaque type representing an ICU Break iterator object.
     *  @stable ICU 2.0
     */
    typedef void UBreakIterator;
#endif

#if !UCONFIG_NO_BREAK_ITERATION

#include "unicode/parseerr.h"


/** The possible types of text boundaries.  @stable ICU 2.0 */
typedef enum UBreakIteratorType {
  /** Character breaks  @stable ICU 2.0 */
  UBRK_CHARACTER,
  /** Word breaks @stable ICU 2.0 */
  UBRK_WORD,
  /** Line breaks @stable ICU 2.0 */
  UBRK_LINE,
  /** Sentence breaks @stable ICU 2.0 */
  UBRK_SENTENCE,

#ifndef U_HIDE_DEPRECATED_API
  /** 
   * Title Case breaks 
   * The iterator created using this type locates title boundaries as described for 
   * Unicode 3.2 only. For Unicode 4.0 and above title boundary iteration,
   * please use Word Boundary iterator.
   *
   * @deprecated ICU 2.8 Use the word break iterator for titlecasing for Unicode 4 and later.
   */
  UBRK_TITLE
#endif /* U_HIDE_DEPRECATED_API */

} UBreakIteratorType;

#define UBRK_DONE ((int32_t) -1)


typedef enum UWordBreak {
    /** Tag value for "words" that do not fit into any of other categories. 
     *  Includes spaces and most punctuation. */
    UBRK_WORD_NONE           = 0,
    /** Upper bound for tags for uncategorized words. */
    UBRK_WORD_NONE_LIMIT     = 100,
    /** Tag value for words that appear to be numbers, lower limit.    */
    UBRK_WORD_NUMBER         = 100,
    /** Tag value for words that appear to be numbers, upper limit.    */
    UBRK_WORD_NUMBER_LIMIT   = 200,
    /** Tag value for words that contain letters, excluding
     *  hiragana, katakana or ideographic characters, lower limit.    */
    UBRK_WORD_LETTER         = 200,
    /** Tag value for words containing letters, upper limit  */
    UBRK_WORD_LETTER_LIMIT   = 300,
    /** Tag value for words containing kana characters, lower limit */
    UBRK_WORD_KANA           = 300,
    /** Tag value for words containing kana characters, upper limit */
    UBRK_WORD_KANA_LIMIT     = 400,
    /** Tag value for words containing ideographic characters, lower limit */
    UBRK_WORD_IDEO           = 400,
    /** Tag value for words containing ideographic characters, upper limit */
    UBRK_WORD_IDEO_LIMIT     = 500
} UWordBreak;

typedef enum ULineBreakTag {
    /** Tag value for soft line breaks, positions at which a line break
      *  is acceptable but not required                */
    UBRK_LINE_SOFT            = 0,
    /** Upper bound for soft line breaks.              */
    UBRK_LINE_SOFT_LIMIT      = 100,
    /** Tag value for a hard, or mandatory line break  */
    UBRK_LINE_HARD            = 100,
    /** Upper bound for hard line breaks.              */
    UBRK_LINE_HARD_LIMIT      = 200
} ULineBreakTag;



typedef enum USentenceBreakTag {
    /** Tag value for for sentences  ending with a sentence terminator
      * ('.', '?', '!', etc.) character, possibly followed by a
      * hard separator (CR, LF, PS, etc.)
      */
    UBRK_SENTENCE_TERM       = 0,
    /** Upper bound for tags for sentences ended by sentence terminators.    */
    UBRK_SENTENCE_TERM_LIMIT = 100,
    /** Tag value for for sentences that do not contain an ending
      * sentence terminator ('.', '?', '!', etc.) character, but 
      * are ended only by a hard separator (CR, LF, PS, etc.) or end of input.
      */
    UBRK_SENTENCE_SEP        = 100,
    /** Upper bound for tags for sentences ended by a separator.              */
    UBRK_SENTENCE_SEP_LIMIT  = 200
    /** Tag value for a hard, or mandatory line break  */
} USentenceBreakTag;


U_STABLE UBreakIterator* U_EXPORT2
ubrk_open(UBreakIteratorType type,
      const char *locale,
      const UChar *text,
      int32_t textLength,
      UErrorCode *status);

U_STABLE UBreakIterator* U_EXPORT2
ubrk_openRules(const UChar     *rules,
               int32_t         rulesLength,
               const UChar     *text,
               int32_t          textLength,
               UParseError     *parseErr,
               UErrorCode      *status);

U_STABLE UBreakIterator * U_EXPORT2
ubrk_safeClone(
          const UBreakIterator *bi,
          void *stackBuffer,
          int32_t *pBufferSize,
          UErrorCode *status);

#define U_BRK_SAFECLONE_BUFFERSIZE 512

U_STABLE void U_EXPORT2
ubrk_close(UBreakIterator *bi);

U_STABLE void U_EXPORT2
ubrk_setText(UBreakIterator* bi,
             const UChar*    text,
             int32_t         textLength,
             UErrorCode*     status);

U_STABLE int32_t U_EXPORT2
ubrk_current(const UBreakIterator *bi);

U_STABLE int32_t U_EXPORT2
ubrk_next(UBreakIterator *bi);

U_STABLE int32_t U_EXPORT2
ubrk_previous(UBreakIterator *bi);

U_STABLE int32_t U_EXPORT2
ubrk_first(UBreakIterator *bi);

U_STABLE int32_t U_EXPORT2
ubrk_last(UBreakIterator *bi);

U_STABLE int32_t U_EXPORT2
ubrk_preceding(UBreakIterator *bi,
           int32_t offset);

U_STABLE int32_t U_EXPORT2
ubrk_following(UBreakIterator *bi,
           int32_t offset);

U_STABLE const char* U_EXPORT2
ubrk_getAvailable(int32_t index);

U_STABLE int32_t U_EXPORT2
ubrk_countAvailable(void);


U_STABLE  UBool U_EXPORT2
ubrk_isBoundary(UBreakIterator *bi, int32_t offset);

U_STABLE  int32_t U_EXPORT2
ubrk_getRuleStatus(UBreakIterator *bi);

U_DRAFT  int32_t U_EXPORT2
ubrk_getRuleStatusVec(UBreakIterator *bi, int32_t *fillInVec, int32_t capacity, UErrorCode *status);

U_DRAFT const char* U_EXPORT2
ubrk_getLocaleByType(const UBreakIterator *bi, ULocDataLocaleType type, UErrorCode* status);


#endif /* #if !UCONFIG_NO_BREAK_ITERATION */

#endif
