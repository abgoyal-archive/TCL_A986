
#ifndef __PLAYOUT_H
#define __PLAYOUT_H

#include "unicode/ubidi.h"
#if ! UCONFIG_NO_BREAK_ITERATION

#include "layout/LETypes.h"
#include "plruns.h"


typedef void pl_paragraph;

typedef void pl_line;

typedef void pl_visualRun;

U_INTERNAL pl_paragraph * U_EXPORT2
pl_create(const LEUnicode chars[],
          le_int32 count,
          const pl_fontRuns *fontRuns,
          const pl_valueRuns *levelRuns,
          const pl_valueRuns *scriptRuns,
          const pl_localeRuns *localeRuns,
          UBiDiLevel paragraphLevel,
          le_bool vertical,
          LEErrorCode *status);

U_INTERNAL void U_EXPORT2
pl_close(pl_paragraph *paragraph);


U_INTERNAL le_bool U_EXPORT2
pl_isComplex(const LEUnicode chars[],
          le_int32 count);

U_INTERNAL UBiDiLevel U_EXPORT2
pl_getParagraphLevel(pl_paragraph *paragraph);

U_INTERNAL UBiDiDirection U_EXPORT2
pl_getTextDirection(pl_paragraph *paragraph);

U_INTERNAL le_int32 U_EXPORT2
pl_getAscent(const pl_paragraph *paragraph);

U_INTERNAL le_int32 U_EXPORT2
pl_getDescent(const pl_paragraph *paragraph);

U_INTERNAL le_int32 U_EXPORT2
pl_getLeading(const pl_paragraph *paragraph);

U_INTERNAL void U_EXPORT2
pl_reflow(pl_paragraph *paragraph);

U_INTERNAL pl_line * U_EXPORT2
pl_nextLine(pl_paragraph *paragraph, float width);

U_INTERNAL void U_EXPORT2
pl_closeLine(pl_line *line);

U_INTERNAL le_int32 U_EXPORT2
pl_countLineRuns(const pl_line *line);

U_INTERNAL le_int32 U_EXPORT2
pl_getLineAscent(const pl_line *line);

U_INTERNAL le_int32 U_EXPORT2
pl_getLineDescent(const pl_line *line);

U_INTERNAL le_int32 U_EXPORT2
pl_getLineLeading(const pl_line *line);

U_INTERNAL le_int32 U_EXPORT2
pl_getLineWidth(const pl_line *line);

U_INTERNAL const pl_visualRun * U_EXPORT2
pl_getLineVisualRun(const pl_line *line, le_int32 runIndex);

U_INTERNAL const le_font * U_EXPORT2
pl_getVisualRunFont(const pl_visualRun *run);

U_INTERNAL UBiDiDirection U_EXPORT2
pl_getVisualRunDirection(const pl_visualRun *run);

U_INTERNAL le_int32 U_EXPORT2
pl_getVisualRunGlyphCount(const pl_visualRun *run);

U_INTERNAL const LEGlyphID * U_EXPORT2
pl_getVisualRunGlyphs(const pl_visualRun *run);

U_INTERNAL const float * U_EXPORT2
pl_getVisualRunPositions(const pl_visualRun *run);

U_INTERNAL const le_int32 * U_EXPORT2
pl_getVisualRunGlyphToCharMap(const pl_visualRun *run);

U_INTERNAL le_int32 U_EXPORT2
pl_getVisualRunAscent(const pl_visualRun *run);

U_INTERNAL le_int32 U_EXPORT2
pl_getVisualRunDescent(const pl_visualRun *run);

U_INTERNAL le_int32 U_EXPORT2
pl_getVisualRunLeading(const pl_visualRun *run);

#endif
#endif
