


#include "config.h"
#include "SimpleFontData.h"

#include "FloatRect.h"
#include "Font.h"
#include "FontCache.h"
#include "FontDescription.h"
#include <wtf/MathExtras.h>
#include <mlang.h>
#include <tchar.h>

namespace WebCore {

extern HDC g_screenDC;

void SimpleFontData::platformInit()
{
    if (!m_platformData.isValid())
        return;

    const TEXTMETRIC& tm = m_platformData.metrics();
    m_isSystemFont = m_platformData.isSystemFont();

    m_ascent = (tm.tmAscent * m_platformData.size() + 36) / 72;
    m_descent = (tm.tmDescent * m_platformData.size() + 36) / 72;
    m_lineGap = (tm.tmExternalLeading * m_platformData.size() + 36) / 72;
    m_lineSpacing = m_ascent + m_descent + m_lineGap;
    m_xHeight = m_ascent * 0.56f;
}

void SimpleFontData::platformDestroy()
{
    delete m_smallCapsFontData;
    m_smallCapsFontData = 0;
}

SimpleFontData* SimpleFontData::smallCapsFontData(const FontDescription& fontDescription) const
{
    if (!m_smallCapsFontData) {
        FontDescription fontDesc(fontDescription);
        fontDesc.setComputedSize(lroundf(0.70f * fontDesc.computedSize()));
        fontDesc.setSpecifiedSize(lroundf(0.70f * fontDesc.specifiedSize()));
        fontDesc.setKeywordSize(lroundf(0.70f * fontDesc.keywordSize()));
        FontPlatformData* result = fontCache()->getCachedFontPlatformData(fontDesc, m_platformData.family());
        if (result)
            m_smallCapsFontData = new SimpleFontData(*result);
    }
    return m_smallCapsFontData;
}

DWORD getKnownFontCodePages(const wchar_t* family);

bool SimpleFontData::containsCharacters(const UChar* characters, int length) const
{
    if (m_platformData.isDisabled())
        return true;

    // FIXME: Microsoft documentation seems to imply that characters can be output using a given font and DC
    // merely by testing code page intersection.  This seems suspect though.  Can't a font only partially
    // cover a given code page?

    // FIXME: in the case that we failed to get the interface, still use the font.
#if defined(IMLANG_FONT_LINK) && (IMLANG_FONT_LINK == 2)
    IMLangFontLink2* langFontLink = fontCache()->getFontLinkInterface();
#else
    IMLangFontLink* langFontLink = fontCache()->getFontLinkInterface();
#endif
    if (!langFontLink)
        return true;

    DWORD fontCodePages = m_platformData.codePages();
    if (!fontCodePages)
        return false;

    DWORD acpCodePages = 0;
    langFontLink->CodePageToCodePages(CP_ACP, &acpCodePages);

    DWORD actualCodePages;
    long numCharactersProcessed;
    while (length) {
        langFontLink->GetStrCodePages(characters, length, acpCodePages, &actualCodePages, &numCharactersProcessed);
        if (actualCodePages && !(actualCodePages & fontCodePages))
            return false;

        length -= numCharactersProcessed;
        characters += numCharactersProcessed;
    }

    return true;
}

void SimpleFontData::determinePitch()
{
    if (!m_platformData.isValid())
        return;

    const TEXTMETRIC& tm = m_platformData.metrics();

    // Yes, this looks backwards, but the fixed pitch bit is actually set if the font
    // is *not* fixed pitch.  Unbelievable but true.
    m_treatAsFixedPitch = !(tm.tmPitchAndFamily & TMPF_FIXED_PITCH);
}

float SimpleFontData::platformWidthForGlyph(Glyph glyph) const
{
    if (m_platformData.isDisabled())
        return 0;

    HGDIOBJ hOldFont = SelectObject(g_screenDC, m_platformData.hfont());

    SIZE fontSize;
    wchar_t c = glyph;
    GetTextExtentPoint32(g_screenDC, &c, 1, &fontSize);

    SelectObject(g_screenDC, hOldFont);

    return (float)fontSize.cx * (float)m_platformData.size() / 72.f;
}


void SimpleFontData::platformCharWidthInit()
{
    if (!m_platformData.isValid())
        return;

    const TEXTMETRIC& tm = m_platformData.metrics();
    m_avgCharWidth = (tm.tmAveCharWidth * m_platformData.size() + 36) / 72;
    m_maxCharWidth = (tm.tmMaxCharWidth * m_platformData.size() + 36) / 72;
}

}
