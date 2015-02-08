

#include "config.h"
#include "GlyphPageTreeNode.h"

#include "SimpleFontData.h"

namespace WebCore {

bool GlyphPage::fill(unsigned offset, unsigned length, UChar* buffer, unsigned bufferLength, const SimpleFontData* fontData)
{
    // bufferLength will be greater than the requested number of glyphs if the buffer contains surrogate pairs.
    // We won't support this for now.
    if (bufferLength > length)
        return false;

    bool haveGlyphs = false;

    HDC dc = GetDC((HWND)0);
    SaveDC(dc);
    SelectObject(dc, fontData->platformData().hfont());

    TEXTMETRIC tm;
    GetTextMetrics(dc, &tm);

    WORD localGlyphBuffer[GlyphPage::size * 2];
    DWORD result = GetGlyphIndices(dc, buffer, bufferLength, localGlyphBuffer, 0);
    bool success = result != GDI_ERROR && static_cast<unsigned>(result) == bufferLength;
    if (success) {
        for (unsigned i = 0; i < length; i++) {
            Glyph glyph = localGlyphBuffer[i];
            if (!glyph)
                setGlyphDataForIndex(offset + i, 0, 0);
            else {
                setGlyphDataForIndex(offset + i, glyph, fontData);
                haveGlyphs = true;
            }
        }
    }
    RestoreDC(dc, -1);
    ReleaseDC(0, dc);

    return haveGlyphs;
}

}
