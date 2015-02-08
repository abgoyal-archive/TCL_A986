

#include "config.h"
#include "GlyphPageTreeNode.h"

#include "Font.h"
#include "SimpleFontData.h"

#include "SkTemplates.h"
#include "SkPaint.h"
#include "SkUtils.h"

namespace WebCore {

bool GlyphPage::fill(unsigned offset, unsigned length, UChar* buffer, unsigned bufferLength, const SimpleFontData* fontData)
{
    if (SkUTF16_IsHighSurrogate(buffer[bufferLength-1])) {
        SkDebugf("%s last char is high-surrogate", __FUNCTION__);
        return false;
    }
    
    SkPaint paint;
    fontData->platformData().setupPaint(&paint);
    paint.setTextEncoding(SkPaint::kUTF16_TextEncoding);
    
    SkAutoSTMalloc <GlyphPage::size, uint16_t> glyphStorage(length);
    uint16_t* glyphs = glyphStorage.get();
    // textToGlyphs takes a byte count, not a glyph count so we multiply by two.
    unsigned count = paint.textToGlyphs(buffer, bufferLength * 2, glyphs);
    if (count != length) {
        SkDebugf("%s count != length\n", __FUNCTION__);
        return false;
    }

    unsigned allGlyphs = 0; // track if any of the glyphIDs are non-zero
    for (unsigned i = 0; i < length; i++) {
        setGlyphDataForIndex(offset + i, glyphs[i], glyphs[i] ? fontData : NULL);
        allGlyphs |= glyphs[i];
    }

    return allGlyphs != 0;
}

}  // namespace WebCore
