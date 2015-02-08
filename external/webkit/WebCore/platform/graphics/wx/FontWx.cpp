

#include "config.h"
#include "Font.h"

#include "FontFallbackList.h"
#include "GlyphBuffer.h"
#include "GraphicsContext.h"
#include "IntRect.h"
#include "NotImplemented.h"
#include "SimpleFontData.h"

#if OS(WINDOWS)
#include "UniscribeController.h"
#endif

#include <wx/dcclient.h>
#include "fontprops.h"
#include "non-kerned-drawing.h"

namespace WebCore {

bool Font::canReturnFallbackFontsForComplexText()
{
#if OS(WINDOWS)
    return true;
#else
    return false;
#endif
}

void Font::drawGlyphs(GraphicsContext* graphicsContext, const SimpleFontData* font, const GlyphBuffer& glyphBuffer, 
                      int from, int numGlyphs, const FloatPoint& point) const
{
    // prepare DC
    Color color = graphicsContext->fillColor();

    // We can't use wx drawing methods on Win/Linux because they automatically kern text
    // so we've created a function with platform dependent drawing implementations that
    // will hopefully be folded into wx once the API has solidified.
    // see platform/wx/wxcode/<platform> for the implementations.
    drawTextWithSpacing(graphicsContext, font, color, glyphBuffer, from, numGlyphs, point);
}

FloatRect Font::selectionRectForComplexText(const TextRun& run, const IntPoint& point, int h, int from, int to) const
{
#if OS(WINDOWS)
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
#else
    notImplemented();
    return FloatRect();
#endif
}

void Font::drawComplexText(GraphicsContext* context, const TextRun& run, const FloatPoint& point, int from, int to) const
{
#if OS(WINDOWS)
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
#else
    notImplemented();
#endif
}


float Font::floatWidthForComplexText(const TextRun& run, HashSet<const SimpleFontData*>* fallbackFonts) const
{
#if OS(WINDOWS)
    UniscribeController controller(this, run, fallbackFonts);
    controller.advance(run.length());
    return controller.runWidthSoFar();
#else
    notImplemented();
    return 0;
#endif
}

int Font::offsetForPositionForComplexText(const TextRun& run, int x, bool includePartialGlyphs) const
{
#if OS(WINDOWS)
    UniscribeController controller(this, run);
    return controller.offsetForPosition(x, includePartialGlyphs);
#else
    notImplemented();
    return 0;
#endif
}
}
