
#ifndef __LOENGINE_H
#define __LOENGINE_H

#include "LETypes.h"


typedef void le_engine;

typedef void le_font;

U_INTERNAL le_engine * U_EXPORT2
le_create(const le_font *font,
          le_int32 scriptCode,
          le_int32 languageCode,
          le_int32 typo_flags,
          LEErrorCode *success);

U_INTERNAL void U_EXPORT2
le_close(le_engine *engine);

U_INTERNAL le_int32 U_EXPORT2
le_layoutChars(le_engine *engine,
               const LEUnicode chars[],
               le_int32 offset,
               le_int32 count,
               le_int32 max,
               le_bool rightToLeft,
               float x,
               float y,
               LEErrorCode *success);

U_INTERNAL le_int32 U_EXPORT2
le_getGlyphCount(le_engine *engine,
                 LEErrorCode *success);

U_INTERNAL void U_EXPORT2
le_getGlyphs(le_engine *engine,
             LEGlyphID glyphs[],
             LEErrorCode *success);

U_INTERNAL void U_EXPORT2
le_getCharIndices(le_engine *engine,
                  le_int32 charIndices[],
                  LEErrorCode *success);

U_INTERNAL void U_EXPORT2
le_getCharIndicesWithBase(le_engine *engine,
                  le_int32 charIndices[],
                  le_int32 indexBase,
                  LEErrorCode *success);

U_INTERNAL void U_EXPORT2
le_getGlyphPositions(le_engine *engine,
                     float positions[],
                     LEErrorCode *success);

U_INTERNAL void U_EXPORT2
le_getGlyphPosition(le_engine *engine,
                    le_int32 glyphIndex,
                    float *x,
                    float *y,
                    LEErrorCode *success);

U_INTERNAL void U_EXPORT2
le_reset(le_engine *engine,
         LEErrorCode *success);

#endif
