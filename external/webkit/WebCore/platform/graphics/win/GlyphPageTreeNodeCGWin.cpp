

#include "config.h"
#include "GlyphPageTreeNode.h"

#include "SimpleFontData.h"
#include <WebKitSystemInterface/WebKitSystemInterface.h>

namespace WebCore {

bool GlyphPage::fill(unsigned offset, unsigned length, UChar* buffer, unsigned bufferLength, const SimpleFontData* fontData)
{
    // bufferLength will be greater than the requested number of glyphs if the buffer contains surrogate pairs.
    // We won't support this for now.
    if (bufferLength > length)
        return false;

    bool haveGlyphs = false;
    CGGlyph localGlyphBuffer[GlyphPage::size];
    wkGetGlyphs(fontData->platformData().cgFont(), buffer, localGlyphBuffer, bufferLength);
    for (unsigned i = 0; i < length; i++) {
        Glyph glyph = localGlyphBuffer[i];
        if (!glyph)
            setGlyphDataForIndex(offset + i, 0, 0);
        else {
            setGlyphDataForIndex(offset + i, glyph, fontData);
            haveGlyphs = true;
        }
    }
    return haveGlyphs;
}

}
