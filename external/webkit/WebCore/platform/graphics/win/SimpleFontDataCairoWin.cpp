

#include "config.h"
#include "SimpleFontData.h"

#include <windows.h>

#include "Font.h"
#include "FontCache.h"
#include "FontDescription.h"
#include <cairo.h>
#include <cairo-win32.h>
#include <mlang.h>
#include <tchar.h>
#include <wtf/MathExtras.h>

namespace WebCore {

void SimpleFontData::platformInit()
{
    m_scriptCache = 0;
    m_scriptFontProperties = 0;
    m_isSystemFont = false;

    m_syntheticBoldOffset = m_platformData.syntheticBold() ? 1.0f : 0.f;

    if (m_platformData.useGDI())
       return initGDIFont();

    HDC hdc = GetDC(0);
    SaveDC(hdc);

    cairo_scaled_font_t* scaledFont = m_platformData.scaledFont();
    const double metricsMultiplier = cairo_win32_scaled_font_get_metrics_factor(scaledFont) * m_platformData.size();

    cairo_win32_scaled_font_select_font(scaledFont, hdc);

    TEXTMETRIC textMetrics;
    GetTextMetrics(hdc, &textMetrics);
    m_ascent = lroundf(textMetrics.tmAscent * metricsMultiplier);
    m_descent = lroundf(textMetrics.tmDescent * metricsMultiplier);
    m_xHeight = m_ascent * 0.56f; // Best guess for xHeight for non-Truetype fonts.
    m_lineGap = lroundf(textMetrics.tmExternalLeading * metricsMultiplier);
    m_lineSpacing = m_ascent + m_descent + m_lineGap;
    m_avgCharWidth = lroundf(textMetrics.tmAveCharWidth * metricsMultiplier);
    m_maxCharWidth = lroundf(textMetrics.tmMaxCharWidth * metricsMultiplier);

    OUTLINETEXTMETRIC metrics;
    if (GetOutlineTextMetrics(hdc, sizeof(metrics), &metrics) > 0) {
        // This is a TrueType font.  We might be able to get an accurate xHeight
        GLYPHMETRICS gm;
        MAT2 mat = { 1, 0, 0, 1 };
        DWORD len = GetGlyphOutline(hdc, 'x', GGO_METRICS, &gm, 0, 0, &mat);
        if (len != GDI_ERROR && gm.gmptGlyphOrigin.y > 0)
            m_xHeight = gm.gmptGlyphOrigin.y * metricsMultiplier;
    }

    cairo_win32_scaled_font_done_font(scaledFont);

    m_isSystemFont = false;
    m_scriptCache = 0;
    m_scriptFontProperties = 0;

    RestoreDC(hdc, -1);
    ReleaseDC(0, hdc);
}

void SimpleFontData::platformCharWidthInit()
{
    // charwidths are set in platformInit.
}

float SimpleFontData::platformWidthForGlyph(Glyph glyph) const
{
    if (m_platformData.useGDI())
       return widthForGDIGlyph(glyph);

    HDC hdc = GetDC(0);
    SaveDC(hdc);

    cairo_scaled_font_t* scaledFont = m_platformData.scaledFont();
    cairo_win32_scaled_font_select_font(scaledFont, hdc);

    int width;
    GetCharWidthI(hdc, glyph, 1, 0, &width);

    cairo_win32_scaled_font_done_font(scaledFont);

    RestoreDC(hdc, -1);
    ReleaseDC(0, hdc);

    const double metricsMultiplier = cairo_win32_scaled_font_get_metrics_factor(scaledFont) * m_platformData.size();
    return width * metricsMultiplier;
}

}
