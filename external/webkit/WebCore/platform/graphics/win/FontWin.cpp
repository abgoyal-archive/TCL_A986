

#include "config.h"
#include "Font.h"

#include "FontFallbackList.h"
#include "GlyphBuffer.h"
#include "GraphicsContext.h"
#include "IntRect.h"
#include "SimpleFontData.h"
#include "UniscribeController.h"
#include <wtf/MathExtras.h>

namespace WebCore {

bool Font::canReturnFallbackFontsForComplexText()
{
    return true;
}

FloatRect Font::selectionRectForComplexText(const TextRun& run, const IntPoint& point, int h,
                                            int from, int to) const
{
    UniscribeController it(this, run);
    it.advance(from);
    float beforeWidth = it.runWidthSoFar();
    it.advance(to);
    float afterWidth = it.runWidthSoFar();

    // Using roundf() rather than ceilf() for the right edge as a compromise to ensure correct caret positioning
    if (run.rtl()) {
        it.advance(run.length());
        float totalWidth = it.runWidthSoFar();
        return FloatRect(point.x() + floorf(totalWidth - afterWidth), point.y(), roundf(totalWidth - beforeWidth) - floorf(totalWidth - afterWidth), h);
    } 
    
    return FloatRect(point.x() + floorf(beforeWidth), point.y(), roundf(afterWidth) - floorf(beforeWidth), h);
}

void Font::drawComplexText(GraphicsContext* context, const TextRun& run, const FloatPoint& point,
                           int from, int to) const
{
    // This glyph buffer holds our glyphs + advances + font data for each glyph.
    GlyphBuffer glyphBuffer;

    float startX = point.x();
    UniscribeController controller(this, run);
    controller.advance(from);
    float beforeWidth = controller.runWidthSoFar();
    controller.advance(to, &glyphBuffer);
    
    // We couldn't generate any glyphs for the run.  Give up.
    if (glyphBuffer.isEmpty())
        return;
    
    float afterWidth = controller.runWidthSoFar();

    if (run.rtl()) {
        controller.advance(run.length());
        startX += controller.runWidthSoFar() - afterWidth;
    } else
        startX += beforeWidth;

    // Draw the glyph buffer now at the starting point returned in startX.
    FloatPoint startPoint(startX, point.y());
    drawGlyphBuffer(context, glyphBuffer, run, startPoint);
}

float Font::floatWidthForComplexText(const TextRun& run, HashSet<const SimpleFontData*>* fallbackFonts) const
{
    UniscribeController controller(this, run, fallbackFonts);
    controller.advance(run.length());
    return controller.runWidthSoFar();
}

int Font::offsetForPositionForComplexText(const TextRun& run, int x, bool includePartialGlyphs) const
{
    UniscribeController controller(this, run);
    return controller.offsetForPosition(x, includePartialGlyphs);
}

}
