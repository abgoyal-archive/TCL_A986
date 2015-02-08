

#include "config.h"
#include "SimpleFontData.h"

#include <QFontMetrics>

namespace WebCore {

void SimpleFontData::determinePitch()
{
    m_treatAsFixedPitch = m_platformData.font().fixedPitch();
}

bool SimpleFontData::containsCharacters(const UChar*, int) const
{
    return true;
}

void SimpleFontData::platformInit()
{
    QFontMetrics fm(m_platformData.font());

    m_ascent = fm.ascent();
    m_descent = fm.descent();
    m_lineSpacing = fm.lineSpacing();
    m_xHeight = fm.xHeight();
    m_spaceWidth = fm.width(QLatin1Char(' '));
    m_lineGap = fm.leading();
}

void SimpleFontData::platformGlyphInit()
{
    m_spaceGlyph = 0;
    m_adjustedSpaceWidth = m_spaceWidth;
    determinePitch();
    m_missingGlyphData.fontData = this;
    m_missingGlyphData.glyph = 0;
}

void SimpleFontData::platformCharWidthInit()
{
    QFontMetrics fm(m_platformData.font());
    m_avgCharWidth = fm.averageCharWidth();
    m_maxCharWidth = fm.maxWidth();
}

void SimpleFontData::platformDestroy()
{
}

}
