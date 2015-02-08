

#include "config.h"
#include "GlyphPageTreeNode.h"

#include "Font.h"
#include "FontCache.h"
#include "FontData.h"
#include "SimpleFontData.h"

namespace WebCore {

DWORD getKnownFontCodePages(const wchar_t* family);

typedef unsigned (*funcGetCharCodePages)(unsigned short c, unsigned& lastPos);
funcGetCharCodePages getCharCodePages = 0;

bool GlyphPage::fill(unsigned offset, unsigned length, UChar* buffer, unsigned bufferLength, const SimpleFontData* fontData)
{
    if (length != bufferLength)
        return false;

    if (fontData->platformData().hfont()) {
        DWORD fontCodePages = fontData->platformData().codePages();
        if (fontCodePages) {
            if (getCharCodePages) {
                unsigned lastPos = 0;
                for (unsigned i = 0; i < bufferLength; ++i) {
                    DWORD actualCodePages = getCharCodePages(buffer[i], lastPos);
                    if (!actualCodePages || (actualCodePages & fontCodePages))
                        setGlyphDataForIndex(offset + i, buffer[i], fontData);
                    else
                        setGlyphDataForIndex(offset + i, buffer[i], 0);
                }
                return true;
#if defined(IMLANG_FONT_LINK) && (IMLANG_FONT_LINK == 2)
            } else if (IMLangFontLink2* langFontLink = fontCache()->getFontLinkInterface()) {
#else
            } else if (IMLangFontLink* langFontLink = fontCache()->getFontLinkInterface()) {
#endif
                for (unsigned i = 0; i < bufferLength; ++i) {
                    DWORD actualCodePages;
                    langFontLink->GetCharCodePages(buffer[i], &actualCodePages);
                    if (!actualCodePages || (actualCodePages & fontCodePages))
                        setGlyphDataForIndex(offset + i, buffer[i], fontData);
                    else
                        setGlyphDataForIndex(offset + i, buffer[i], 0);
                }
                return true;
            }
        }
    }

    for (unsigned i = 0; i < length; ++i)
        setGlyphDataForIndex(offset + i, buffer[i], fontData);

    return true;
}

}

