

#include "config.h"
#include "SimpleFontData.h"

#include "FontCache.h"
#include "FloatRect.h"
#include "FontDescription.h"
#include <wtf/MathExtras.h>
#include <unicode/uchar.h>
#include <unicode/unorm.h>

#include <wx/defs.h>
#include <wx/dcscreen.h>
#include "fontprops.h"

namespace WebCore
{

void SimpleFontData::platformInit()
{    
    wxFont *font = m_platformData.font();
    if (font && font->IsOk()) {
        wxFontProperties props = wxFontProperties(font);
        m_ascent = props.GetAscent();
        m_descent = props.GetDescent();
        m_lineSpacing = props.GetLineSpacing();
        m_xHeight = props.GetXHeight();
        m_unitsPerEm = 1; // FIXME!
        m_lineGap = props.GetLineGap();
    }

#if OS(WINDOWS)
    m_scriptCache = 0;
    m_scriptFontProperties = 0;
    m_isSystemFont = false;
    m_syntheticBoldOffset = 0.0f;
#endif
}

void SimpleFontData::platformCharWidthInit()
{
    m_avgCharWidth = 0.f;
    m_maxCharWidth = 0.f;
    initCharWidths();
}

void SimpleFontData::platformDestroy()
{
    delete m_smallCapsFontData;
    m_smallCapsFontData = 0;
    
#if OS(WINDOWS)
    if (m_scriptFontProperties) {
        delete m_scriptFontProperties;
        m_scriptFontProperties = 0;
    }

    if (m_scriptCache)
        ScriptFreeCache(&m_scriptCache);
#endif
}

SimpleFontData* SimpleFontData::smallCapsFontData(const FontDescription& fontDescription) const
{
    if (!m_smallCapsFontData){
        FontDescription desc = FontDescription(fontDescription);
        desc.setSpecifiedSize(0.70f*fontDescription.computedSize());
        const FontPlatformData* pdata = new FontPlatformData(desc, desc.family().family());
        m_smallCapsFontData = new SimpleFontData(*pdata);
    }
    return m_smallCapsFontData;
}

bool SimpleFontData::containsCharacters(const UChar* characters, int length) const
{
    // FIXME: We will need to implement this to load non-ASCII encoding sites
    return wxFontContainsCharacters(*m_platformData.font(), characters, length);
}

void SimpleFontData::determinePitch()
{
    if (m_platformData.font() && m_platformData.font()->Ok())
        m_treatAsFixedPitch = m_platformData.font()->IsFixedWidth();
    else
        m_treatAsFixedPitch = false;
}

float SimpleFontData::platformWidthForGlyph(Glyph glyph) const
{
#if __WXMSW__
    // under Windows / wxMSW we currently always use GDI fonts.
    return widthForGDIGlyph(glyph);
#else
    // TODO: fix this! Make GetTextExtents a method of wxFont in 2.9
    int width = 10;
    GetTextExtent(*m_platformData.font(), (wxChar)glyph, &width, NULL);
    return width;
#endif
}

#if OS(WINDOWS)
SCRIPT_FONTPROPERTIES* SimpleFontData::scriptFontProperties() const
{
    // AFAICT this is never called even by the Win port anymore.
    return 0;
}

void SimpleFontData::initGDIFont()
{
    // unused by wx port
}

void SimpleFontData::platformCommonDestroy()
{
    // unused by wx port
}

float SimpleFontData::widthForGDIGlyph(Glyph glyph) const
{
    HDC hdc = GetDC(0);
    HGDIOBJ oldFont = SelectObject(hdc, m_platformData.hfont());
    int width;
    GetCharWidthI(hdc, glyph, 1, 0, &width);
    SelectObject(hdc, oldFont);
    ReleaseDC(0, hdc);
    return width;
}
#endif

}
