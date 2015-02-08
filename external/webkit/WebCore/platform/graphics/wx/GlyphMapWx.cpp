

#include "config.h"
#include "GlyphPageTreeNode.h"

#include "SimpleFontData.h"
#include <unicode/utf16.h>

namespace WebCore
{

bool GlyphPage::fill(unsigned offset, unsigned length, UChar* buffer, unsigned bufferLength, const SimpleFontData* fontData)
{
    bool isUtf16 = bufferLength != length;

    for (unsigned i = 0; i < length; i++) {
        UChar32 character;

        if(isUtf16) {
            UChar lead = buffer[i * 2];
            UChar trail = buffer[i * 2 + 1];
            character = U16_GET_SUPPLEMENTARY(lead, trail);
        } else {
            character = buffer[i];
        }

        setGlyphDataForIndex(offset + i, character, fontData);
    }

    return true;
}

}