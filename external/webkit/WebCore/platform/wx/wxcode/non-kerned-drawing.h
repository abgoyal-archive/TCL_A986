

#include "Font.h"
#include "GlyphBuffer.h"

#include <wx/defs.h>
#include <wx/dcclient.h>

#if __WXGTK__ && USE(WXGC)
#include <cairo.h>
#include <pango/pango.h>
#include <pango/pangocairo.h>
#endif

namespace WebCore {

extern void drawTextWithSpacing(GraphicsContext* graphicsContext, const SimpleFontData* font, const wxColour& color, const GlyphBuffer& glyphBuffer, int from, int numGlyphs, const FloatPoint& point);

#if __WXGTK__ && USE(WXGC)
PangoFontMap* pangoFontMap();

PangoFont* createPangoFontForFont(const wxFont* wxfont);
cairo_scaled_font_t* createScaledFontForFont(const wxFont* wxfont);
PangoGlyph pango_font_get_glyph(PangoFont* font, PangoContext* context, gunichar wc);

#endif
}

