

#include "config.h"
#include "SimpleFontData.h"

#include "FloatRect.h"
#include "FontCache.h"
#include "FontDescription.h"
#include "NotImplemented.h"
#include <Rect.h>
#include <unicode/uchar.h>
#include <unicode/unorm.h>


extern int charUnicodeToUTF8HACK(unsigned short, char*);

namespace WebCore {

void SimpleFontData::platformInit()
{
    BFont* font = m_platformData.font();
    if (!font)
        return;

    font_height height;
    font->GetHeight(&height);
    m_ascent = static_cast<int>(height.ascent);
    m_descent = static_cast<int>(height.descent);
    m_lineSpacing = m_ascent + m_descent;
    m_xHeight = height.ascent * 0.56f; // Hack taken from the win port.
    m_lineGap = height.leading;
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
}

SimpleFontData* SimpleFontData::smallCapsFontData(const FontDescription& fontDescription) const
{
    if (!m_smallCapsFontData) {
        FontDescription desc = FontDescription(fontDescription);
        desc.setSpecifiedSize(0.70f * fontDescription.computedSize());
        const FontPlatformData* fontPlatformData = new FontPlatformData(desc, desc.family().family());
        m_smallCapsFontData = new SimpleFontData(*fontPlatformData);
    }
    return m_smallCapsFontData;
}

bool SimpleFontData::containsCharacters(const UChar* characters, int length) const
{
    // FIXME: We will need to implement this to load non-ASCII encoding sites
    return true;
}

void SimpleFontData::determinePitch()
{
    m_treatAsFixedPitch = m_platformData.font() && m_platformData.font()->IsFixed();
}

float SimpleFontData::platformWidthForGlyph(Glyph glyph) const
{
    if (!m_platformData.font())
        return 0;

    char charArray[4];
    float escapements[1];

    charUnicodeToUTF8HACK(glyph, charArray);
    m_platformData.font()->GetEscapements(charArray, 1, escapements);
    return escapements[0] * m_platformData.font()->Size();
}

} // namespace WebCore

