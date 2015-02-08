

#include "config.h"
#include "SimpleFontData.h"

#include "FloatRect.h"
#include "Font.h"
#include "FontCache.h"
#include "FontDescription.h"
#include "GlyphBuffer.h"
#include <cairo.h>
#include <wtf/MathExtras.h>

namespace WebCore {

void SimpleFontData::platformInit()
{
    cairo_font_extents_t font_extents;
    cairo_text_extents_t text_extents;
    cairo_scaled_font_extents(m_platformData.m_scaledFont, &font_extents);
    m_ascent = static_cast<int>(lroundf(font_extents.ascent));
    m_descent = static_cast<int>(lroundf(font_extents.descent));
    m_lineSpacing = static_cast<int>(lroundf(font_extents.height));
    // There seems to be some rounding error in cairo (or in how we
    // use cairo) with some fonts, like DejaVu Sans Mono, which makes
    // cairo report a height smaller than ascent + descent, which is
    // wrong and confuses WebCore's layout system. Workaround this
    // while we figure out what's going on.
    if (m_lineSpacing < m_ascent + m_descent)
        m_lineSpacing = m_ascent + m_descent;
    cairo_scaled_font_text_extents(m_platformData.m_scaledFont, "x", &text_extents);
    m_xHeight = text_extents.height;
    cairo_scaled_font_text_extents(m_platformData.m_scaledFont, " ", &text_extents);
    m_spaceWidth =  static_cast<float>(text_extents.x_advance);
    m_lineGap = m_lineSpacing - m_ascent - m_descent;
    m_syntheticBoldOffset = m_platformData.syntheticBold() ? 1.0f : 0.f;
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
        desc.setSpecifiedSize(0.70f*fontDescription.computedSize());
        const FontPlatformData* pdata = new FontPlatformData(desc, desc.family().family());
        m_smallCapsFontData = new SimpleFontData(*pdata);
    }
    return m_smallCapsFontData;
}

bool SimpleFontData::containsCharacters(const UChar* characters, int length) const
{
    bool result = true;

    PangoCoverage* coverage = pango_font_get_coverage(m_platformData.m_font, pango_language_get_default());

    for (int i = 0; i < length; i++) {
        if (PANGO_COVERAGE_NONE == pango_coverage_get(coverage, characters[i])) {
            result = false;
            break;
        }
    }

    pango_coverage_unref(coverage);

    return result;
}

void SimpleFontData::determinePitch()
{
    if (isCustomFont()) {
        m_treatAsFixedPitch = false;
        return;
    }

    m_treatAsFixedPitch = m_platformData.isFixedPitch();
}

float SimpleFontData::platformWidthForGlyph(Glyph glyph) const
{
    ASSERT(m_platformData.m_scaledFont);

    cairo_glyph_t cglyph = { glyph, 0, 0 };
    cairo_text_extents_t extents;
    cairo_scaled_font_glyph_extents(m_platformData.m_scaledFont, &cglyph, 1, &extents);

    float w = (float)m_spaceWidth;
    if (cairo_scaled_font_status(m_platformData.m_scaledFont) == CAIRO_STATUS_SUCCESS && extents.x_advance != 0)
        w = (float)extents.x_advance;
    return w;
}

}
