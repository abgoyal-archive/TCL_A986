
#ifndef UBIDI_H
#define UBIDI_H

#include "unicode/utypes.h"
#include "unicode/uchar.h"
#include "unicode/localpointer.h"


/*DOCXX_TAG*/
/*@{*/

typedef uint8_t UBiDiLevel;

#define UBIDI_DEFAULT_LTR 0xfe

#define UBIDI_DEFAULT_RTL 0xff

#define UBIDI_MAX_EXPLICIT_LEVEL 61

#define UBIDI_LEVEL_OVERRIDE 0x80

#define UBIDI_MAP_NOWHERE   (-1)

enum UBiDiDirection {
    /** All left-to-right text. This is a 0 value. @stable ICU 2.0 */
    UBIDI_LTR,
    /** All right-to-left text. This is a 1 value. @stable ICU 2.0 */
    UBIDI_RTL,
    /** Mixed-directional text. @stable ICU 2.0 */
    UBIDI_MIXED
};

/** @stable ICU 2.0 */
typedef enum UBiDiDirection UBiDiDirection;

struct UBiDi;

/** @stable ICU 2.0 */
typedef struct UBiDi UBiDi;

U_STABLE UBiDi * U_EXPORT2
ubidi_open(void);

U_STABLE UBiDi * U_EXPORT2
ubidi_openSized(int32_t maxLength, int32_t maxRunCount, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ubidi_close(UBiDi *pBiDi);

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

U_DEFINE_LOCAL_OPEN_POINTER(LocalUBiDiPointer, UBiDi, ubidi_close);

U_NAMESPACE_END

#endif

U_STABLE void U_EXPORT2
ubidi_setInverse(UBiDi *pBiDi, UBool isInverse);


U_STABLE UBool U_EXPORT2
ubidi_isInverse(UBiDi *pBiDi);

U_STABLE void U_EXPORT2
ubidi_orderParagraphsLTR(UBiDi *pBiDi, UBool orderParagraphsLTR);

U_STABLE UBool U_EXPORT2
ubidi_isOrderParagraphsLTR(UBiDi *pBiDi);

typedef enum UBiDiReorderingMode {
    /** Regular Logical to Visual Bidi algorithm according to Unicode.
      * This is a 0 value.
      * @stable ICU 3.6 */
    UBIDI_REORDER_DEFAULT = 0,
    /** Logical to Visual algorithm which handles numbers in a way which
      * mimicks the behavior of Windows XP.
      * @stable ICU 3.6 */
    UBIDI_REORDER_NUMBERS_SPECIAL,
    /** Logical to Visual algorithm grouping numbers with adjacent R characters
      * (reversible algorithm).
      * @stable ICU 3.6 */
    UBIDI_REORDER_GROUP_NUMBERS_WITH_R,
    /** Reorder runs only to transform a Logical LTR string to the Logical RTL
      * string with the same display, or vice-versa.<br>
      * If this mode is set together with option
      * <code>#UBIDI_OPTION_INSERT_MARKS</code>, some Bidi controls in the source
      * text may be removed and other controls may be added to produce the
      * minimum combination which has the required display.
      * @stable ICU 3.6 */
    UBIDI_REORDER_RUNS_ONLY,
    /** Visual to Logical algorithm which handles numbers like L
      * (same algorithm as selected by <code>ubidi_setInverse(TRUE)</code>.
      * @see ubidi_setInverse
      * @stable ICU 3.6 */
    UBIDI_REORDER_INVERSE_NUMBERS_AS_L,
    /** Visual to Logical algorithm equivalent to the regular Logical to Visual
      * algorithm.
      * @stable ICU 3.6 */
    UBIDI_REORDER_INVERSE_LIKE_DIRECT,
    /** Inverse Bidi (Visual to Logical) algorithm for the
      * <code>UBIDI_REORDER_NUMBERS_SPECIAL</code> Bidi algorithm.
      * @stable ICU 3.6 */
    UBIDI_REORDER_INVERSE_FOR_NUMBERS_SPECIAL,
    /** Number of values for reordering mode.
      * @stable ICU 3.6 */
    UBIDI_REORDER_COUNT
} UBiDiReorderingMode;

U_STABLE void U_EXPORT2
ubidi_setReorderingMode(UBiDi *pBiDi, UBiDiReorderingMode reorderingMode);

U_STABLE UBiDiReorderingMode U_EXPORT2
ubidi_getReorderingMode(UBiDi *pBiDi);

typedef enum UBiDiReorderingOption {
    /**
     * option value for <code>ubidi_setReorderingOptions</code>:
     * disable all the options which can be set with this function
     * @see ubidi_setReorderingOptions
     * @stable ICU 3.6
     */
    UBIDI_OPTION_DEFAULT = 0,

    /**
     * option bit for <code>ubidi_setReorderingOptions</code>:
     * insert Bidi marks (LRM or RLM) when needed to ensure correct result of
     * a reordering to a Logical order
     *
     * <p>This option must be set or reset before calling
     * <code>ubidi_setPara</code>.</p>
     *
     * <p>This option is significant only with reordering modes which generate
     * a result with Logical order, specifically:</p>
     * <ul>
     *   <li><code>#UBIDI_REORDER_RUNS_ONLY</code></li>
     *   <li><code>#UBIDI_REORDER_INVERSE_NUMBERS_AS_L</code></li>
     *   <li><code>#UBIDI_REORDER_INVERSE_LIKE_DIRECT</code></li>
     *   <li><code>#UBIDI_REORDER_INVERSE_FOR_NUMBERS_SPECIAL</code></li>
     * </ul>
     *
     * <p>If this option is set in conjunction with reordering mode
     * <code>#UBIDI_REORDER_INVERSE_NUMBERS_AS_L</code> or with calling
     * <code>ubidi_setInverse(TRUE)</code>, it implies
     * option <code>#UBIDI_INSERT_LRM_FOR_NUMERIC</code>
     * in calls to function <code>ubidi_writeReordered()</code>.</p>
     *
     * <p>For other reordering modes, a minimum number of LRM or RLM characters
     * will be added to the source text after reordering it so as to ensure
     * round trip, i.e. when applying the inverse reordering mode on the
     * resulting logical text with removal of Bidi marks
     * (option <code>#UBIDI_OPTION_REMOVE_CONTROLS</code> set before calling
     * <code>ubidi_setPara()</code> or option <code>#UBIDI_REMOVE_BIDI_CONTROLS</code>
     * in <code>ubidi_writeReordered</code>), the result will be identical to the
     * source text in the first transformation.
     *
     * <p>This option will be ignored if specified together with option
     * <code>#UBIDI_OPTION_REMOVE_CONTROLS</code>. It inhibits option
     * <code>UBIDI_REMOVE_BIDI_CONTROLS</code> in calls to function
     * <code>ubidi_writeReordered()</code> and it implies option
     * <code>#UBIDI_INSERT_LRM_FOR_NUMERIC</code> in calls to function
     * <code>ubidi_writeReordered()</code> if the reordering mode is
     * <code>#UBIDI_REORDER_INVERSE_NUMBERS_AS_L</code>.</p>
     *
     * @see ubidi_setReorderingMode
     * @see ubidi_setReorderingOptions
     * @stable ICU 3.6
     */
    UBIDI_OPTION_INSERT_MARKS = 1,

    /**
     * option bit for <code>ubidi_setReorderingOptions</code>:
     * remove Bidi control characters
     *
     * <p>This option must be set or reset before calling
     * <code>ubidi_setPara</code>.</p>
     *
     * <p>This option nullifies option <code>#UBIDI_OPTION_INSERT_MARKS</code>.
     * It inhibits option <code>#UBIDI_INSERT_LRM_FOR_NUMERIC</code> in calls
     * to function <code>ubidi_writeReordered()</code> and it implies option
     * <code>#UBIDI_REMOVE_BIDI_CONTROLS</code> in calls to that function.</p>
     *
     * @see ubidi_setReorderingMode
     * @see ubidi_setReorderingOptions
     * @stable ICU 3.6
     */
    UBIDI_OPTION_REMOVE_CONTROLS = 2,

    /**
     * option bit for <code>ubidi_setReorderingOptions</code>:
     * process the output as part of a stream to be continued
     *
     * <p>This option must be set or reset before calling
     * <code>ubidi_setPara</code>.</p>
     *
     * <p>This option specifies that the caller is interested in processing large
     * text object in parts.
     * The results of the successive calls are expected to be concatenated by the
     * caller. Only the call for the last part will have this option bit off.</p>
     *
     * <p>When this option bit is on, <code>ubidi_setPara()</code> may process
     * less than the full source text in order to truncate the text at a meaningful
     * boundary. The caller should call <code>ubidi_getProcessedLength()</code>
     * immediately after calling <code>ubidi_setPara()</code> in order to
     * determine how much of the source text has been processed.
     * Source text beyond that length should be resubmitted in following calls to
     * <code>ubidi_setPara</code>. The processed length may be less than
     * the length of the source text if a character preceding the last character of
     * the source text constitutes a reasonable boundary (like a block separator)
     * for text to be continued.<br>
     * If the last character of the source text constitutes a reasonable
     * boundary, the whole text will be processed at once.<br>
     * If nowhere in the source text there exists
     * such a reasonable boundary, the processed length will be zero.<br>
     * The caller should check for such an occurrence and do one of the following:
     * <ul><li>submit a larger amount of text with a better chance to include
     *         a reasonable boundary.</li>
     *     <li>resubmit the same text after turning off option
     *         <code>UBIDI_OPTION_STREAMING</code>.</li></ul>
     * In all cases, this option should be turned off before processing the last
     * part of the text.</p>
     *
     * <p>When the <code>UBIDI_OPTION_STREAMING</code> option is used,
     * it is recommended to call <code>ubidi_orderParagraphsLTR()</code> with
     * argument <code>orderParagraphsLTR</code> set to <code>TRUE</code> before
     * calling <code>ubidi_setPara</code> so that later paragraphs may be
     * concatenated to previous paragraphs on the right.</p>
     *
     * @see ubidi_setReorderingMode
     * @see ubidi_setReorderingOptions
     * @see ubidi_getProcessedLength
     * @see ubidi_orderParagraphsLTR
     * @stable ICU 3.6
     */
    UBIDI_OPTION_STREAMING = 4
} UBiDiReorderingOption;

U_STABLE void U_EXPORT2
ubidi_setReorderingOptions(UBiDi *pBiDi, uint32_t reorderingOptions);

U_STABLE uint32_t U_EXPORT2
ubidi_getReorderingOptions(UBiDi *pBiDi);

U_STABLE void U_EXPORT2
ubidi_setPara(UBiDi *pBiDi, const UChar *text, int32_t length,
              UBiDiLevel paraLevel, UBiDiLevel *embeddingLevels,
              UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ubidi_setLine(const UBiDi *pParaBiDi,
              int32_t start, int32_t limit,
              UBiDi *pLineBiDi,
              UErrorCode *pErrorCode);

U_STABLE UBiDiDirection U_EXPORT2
ubidi_getDirection(const UBiDi *pBiDi);

U_STABLE const UChar * U_EXPORT2
ubidi_getText(const UBiDi *pBiDi);

U_STABLE int32_t U_EXPORT2
ubidi_getLength(const UBiDi *pBiDi);

U_STABLE UBiDiLevel U_EXPORT2
ubidi_getParaLevel(const UBiDi *pBiDi);

U_STABLE int32_t U_EXPORT2
ubidi_countParagraphs(UBiDi *pBiDi);

U_STABLE int32_t U_EXPORT2
ubidi_getParagraph(const UBiDi *pBiDi, int32_t charIndex, int32_t *pParaStart,
                   int32_t *pParaLimit, UBiDiLevel *pParaLevel,
                   UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ubidi_getParagraphByIndex(const UBiDi *pBiDi, int32_t paraIndex,
                          int32_t *pParaStart, int32_t *pParaLimit,
                          UBiDiLevel *pParaLevel, UErrorCode *pErrorCode);

U_STABLE UBiDiLevel U_EXPORT2
ubidi_getLevelAt(const UBiDi *pBiDi, int32_t charIndex);

U_STABLE const UBiDiLevel * U_EXPORT2
ubidi_getLevels(UBiDi *pBiDi, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ubidi_getLogicalRun(const UBiDi *pBiDi, int32_t logicalPosition,
                    int32_t *pLogicalLimit, UBiDiLevel *pLevel);

U_STABLE int32_t U_EXPORT2
ubidi_countRuns(UBiDi *pBiDi, UErrorCode *pErrorCode);

U_STABLE UBiDiDirection U_EXPORT2
ubidi_getVisualRun(UBiDi *pBiDi, int32_t runIndex,
                   int32_t *pLogicalStart, int32_t *pLength);

U_STABLE int32_t U_EXPORT2
ubidi_getVisualIndex(UBiDi *pBiDi, int32_t logicalIndex, UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
ubidi_getLogicalIndex(UBiDi *pBiDi, int32_t visualIndex, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ubidi_getLogicalMap(UBiDi *pBiDi, int32_t *indexMap, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ubidi_getVisualMap(UBiDi *pBiDi, int32_t *indexMap, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ubidi_reorderLogical(const UBiDiLevel *levels, int32_t length, int32_t *indexMap);

U_STABLE void U_EXPORT2
ubidi_reorderVisual(const UBiDiLevel *levels, int32_t length, int32_t *indexMap);

U_STABLE void U_EXPORT2
ubidi_invertMap(const int32_t *srcMap, int32_t *destMap, int32_t length);

/** option flags for ubidi_writeReordered() */

#define UBIDI_KEEP_BASE_COMBINING       1

#define UBIDI_DO_MIRRORING              2

#define UBIDI_INSERT_LRM_FOR_NUMERIC    4

#define UBIDI_REMOVE_BIDI_CONTROLS      8

#define UBIDI_OUTPUT_REVERSE            16

U_STABLE int32_t U_EXPORT2
ubidi_getProcessedLength(const UBiDi *pBiDi);

U_STABLE int32_t U_EXPORT2
ubidi_getResultLength(const UBiDi *pBiDi);

U_CDECL_BEGIN
#define U_BIDI_CLASS_DEFAULT  U_CHAR_DIRECTION_COUNT

typedef UCharDirection U_CALLCONV
UBiDiClassCallback(const void *context, UChar32 c);

U_CDECL_END

U_STABLE UCharDirection U_EXPORT2
ubidi_getCustomizedClass(UBiDi *pBiDi, UChar32 c);

U_STABLE void U_EXPORT2
ubidi_setClassCallback(UBiDi *pBiDi, UBiDiClassCallback *newFn,
                       const void *newContext, UBiDiClassCallback **oldFn,
                       const void **oldContext, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ubidi_getClassCallback(UBiDi *pBiDi, UBiDiClassCallback **fn, const void **context);

U_STABLE int32_t U_EXPORT2
ubidi_writeReordered(UBiDi *pBiDi,
                     UChar *dest, int32_t destSize,
                     uint16_t options,
                     UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
ubidi_writeReverse(const UChar *src, int32_t srcLength,
                   UChar *dest, int32_t destSize,
                   uint16_t options,
                   UErrorCode *pErrorCode);

/*#define BIDI_SAMPLE_CODE*/
/*@}*/

#endif
