
#ifndef __RENDERINGSURFACE_H
#define __RENDERINGSURFACE_H

#include "layout/LETypes.h"
#include "layout/LEFontInstance.h"

class RenderingSurface
{
public:
    RenderingSurface() {};
    virtual ~RenderingSurface() {};

    virtual void drawGlyphs(const LEFontInstance *font, const LEGlyphID *glyphs, le_int32 count,
                    const float *positions, le_int32 x, le_int32 y, le_int32 width, le_int32 height) = 0;
};

#endif
