

#include "config.h"

#include "EmojiFont.h"
#include "GlyphPageTreeNode.h"
#include "SkTemplates.h"
#include "SkPaint.h"
#include "SkUtils.h"
#include "SimpleFontData.h"

using namespace android;

namespace WebCore {

#define NO_BREAK_SPACE_UNICHAR  0xA0

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
    unsigned count = paint.textToGlyphs(buffer, bufferLength << 1, glyphs);
    if (count != length) {
        SkDebugf("%s count != length\n", __FUNCTION__);
        return false;
    }

    unsigned allGlyphs = 0; // track if any of the glyphIDs are non-zero

    // search for emoji. If we knew for sure that buffer was a contiguous range
    // of chars, we could quick-reject the range to avoid this loop (usually)
    if (EmojiFont::IsAvailable()) {
        const UChar* curr = buffer;
        for (unsigned i = 0; i < length; i++) {
            SkUnichar uni = SkUTF16_NextUnichar(&curr);
            uint16_t glyphID = glyphs[i];
            // only sniff if the normal font failed to recognize it
            if (!glyphID)
                glyphID = EmojiFont::UnicharToGlyph(uni);
            setGlyphDataForIndex(offset + i, glyphID, fontData);
            allGlyphs |= glyphID;
        }
    } else {
        for (unsigned i = 0; i < length; i++) {
            uint16_t glyphID = glyphs[i];
            setGlyphDataForIndex(offset + i, glyphID, fontData);
            allGlyphs |= glyphID;
        }
    }
    return allGlyphs != 0;
}

}
