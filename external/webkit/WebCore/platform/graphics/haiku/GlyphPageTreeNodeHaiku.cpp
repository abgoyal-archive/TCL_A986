

#include "config.h"
#include "GlyphPageTreeNode.h"

#include "SimpleFontData.h"
#include <wtf/Assertions.h>
#include <wtf/unicode/Unicode.h>


namespace WebCore {

bool GlyphPage::fill(unsigned offset, unsigned length, UChar* characterBuffer, unsigned bufferLength, const SimpleFontData* fontData)
{
    bool isUtf16 = bufferLength != GlyphPage::size;
    bool haveGlyphs = false;

    for (unsigned i = 0; i < GlyphPage::size; i++) {
        UChar32 character;

        if (isUtf16) {
            UChar lead = characterBuffer[i * 2];
            UChar trail = characterBuffer[i * 2 + 1];
            character = U16_GET_SUPPLEMENTARY(lead, trail);
        } else
            character = characterBuffer[i];

        if (!character)
            setGlyphDataForIndex(offset + i, 0, 0);
        else {
            haveGlyphs = true;
            setGlyphDataForIndex(offset + i, character, fontData);
        }
    }

    return haveGlyphs;
}

} // namespace WebCore

