

#include "config.h"
#include "GlyphPageTreeNode.h"

#include "SimpleFontData.h"

namespace WebCore {

bool GlyphPage::fill(unsigned offset, unsigned length, UChar* buffer, unsigned bufferLength, const SimpleFontData* fontData)
{
    // The bufferLength will be greater than the glyph page size if the buffer has Unicode supplementary characters.
    // We won't support this for now.
    if (bufferLength > GlyphPage::size)
        return false;

    FT_Face face = cairo_ft_scaled_font_lock_face(fontData->platformData().m_scaledFont);
    if (!face)
        return false;

    bool haveGlyphs = false;
    for (unsigned i = 0; i < length; i++) {
        Glyph glyph = FcFreeTypeCharIndex(face, buffer[i]);
        if (!glyph)
            setGlyphDataForIndex(offset + i, 0, 0);
        else {
            setGlyphDataForIndex(offset + i, glyph, fontData);
            haveGlyphs = true;
        }
    }

    cairo_ft_scaled_font_unlock_face(fontData->platformData().m_scaledFont);

    return haveGlyphs;
}

}
