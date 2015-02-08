

#include "config.h"
#include "Font.h"

#include "FontData.h"
#include "FontDescription.h"
#include "FontSelector.h"
#include "GraphicsContext.h"
#include "NotImplemented.h"
#include <Font.h>
#include <String.h>
#include <View.h>


// FIXME: Temp routine to convert unicode character to UTF8.
int charUnicodeToUTF8HACK(unsigned short glyph, char* out)
{
    int i = 0;

    if (glyph < 0x0080)
        out[i++] = static_cast<char>(glyph);
    else if (glyph < 0x0800) {  // 2 bytes
        out[i++] = 0xc0 | (glyph >> 6);
        out[i++] = 0x80 | (glyph & 0x3F);
    } else if (glyph > 0x0800) {  // 3 bytes
        out[i++] = 0xe0 | (glyph >> 12);
        out[i++] = 0x80 | ((glyph >> 6) & 0x3F);
        out[i++] = 0x80 | (glyph & 0x3F);
    }

    out[i] = '\0';
    return i;
}

namespace WebCore {

bool Font::canReturnFallbackFontsForComplexText()
{
    return false;
}

void Font::drawGlyphs(GraphicsContext* graphicsContext, const SimpleFontData* font,
                      const GlyphBuffer& glyphBuffer, int from, int numGlyphs, const FloatPoint& point) const
{
    Color color = graphicsContext->fillColor();
    BView* view = graphicsContext->platformContext();
    BFont* m_font = font->platformData().font();

    graphicsContext->setCompositeOperation(CompositeSourceOver);
    view->SetHighColor(color);
    view->SetFont(m_font);

    GlyphBufferGlyph* glyphs = const_cast<GlyphBufferGlyph*>(glyphBuffer.glyphs(from));
    float offset = point.x();
    for (int i = 0; i < numGlyphs; i++) {
        char out[4];
        charUnicodeToUTF8HACK(glyphs[i], out);

        view->DrawString(out, sizeof(out), BPoint(offset, point.y()));
        offset += glyphBuffer.advanceAt(from + i);
    }
}

void Font::drawComplexText(GraphicsContext* ctx, const TextRun& run, const FloatPoint& point,
                           int from, int to) const
{
    notImplemented();
}


float Font::floatWidthForComplexText(const TextRun& run, HashSet<const SimpleFontData*>* fallbackFonts) const
{
    notImplemented();
    return 0;
}

FloatRect Font::selectionRectForComplexText(const TextRun&, const IntPoint&, int, int, int) const
{
    notImplemented();
    return FloatRect();
}

int Font::offsetForPositionForComplexText(const TextRun&, int, bool) const
{
    notImplemented();
    return 0;
}

} // namespace WebCore

