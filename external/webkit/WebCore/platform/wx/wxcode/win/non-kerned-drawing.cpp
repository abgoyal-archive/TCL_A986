

#include "config.h"
#include "GlyphBuffer.h"
#include "GraphicsContext.h"
#include "SimpleFontData.h"

#include <wx/defs.h>
#include <wx/dcclient.h>
#include <wx/dcgraph.h>
#include <wx/gdicmn.h>
#include <vector>

using namespace std;

//-----------------------------------------------------------------------------
// constants
//-----------------------------------------------------------------------------

const double RAD2DEG = 180.0 / M_PI;

//-----------------------------------------------------------------------------
// Local functions
//-----------------------------------------------------------------------------

static inline double dmin(double a, double b) { return a < b ? a : b; }
static inline double dmax(double a, double b) { return a > b ? a : b; }

static inline double DegToRad(double deg) { return (deg * M_PI) / 180.0; }
static inline double RadToDeg(double deg) { return (deg * 180.0) / M_PI; }

#include "wx/msw/private.h"

// TODO remove this dependency (gdiplus needs the macros)

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#include "gdiplus.h"
 

namespace WebCore {

void drawTextWithSpacing(GraphicsContext* graphicsContext, const SimpleFontData* font, const wxColour& color, const GlyphBuffer& glyphBuffer, int from, int numGlyphs, const FloatPoint& point)
{
#if USE(WXGC)
    wxGCDC* dc = static_cast<wxGCDC*>(graphicsContext->platformContext());
#else
    wxDC* dc = graphicsContext->platformContext();
#endif

    // get the native HDC handle to draw using native APIs
    HDC hdc = 0;
    float y = point.y() - font->ascent();
    float x = point.x();

#if USE(WXGC)
    // when going from GdiPlus -> Gdi, any GdiPlus transformations are lost
    // so we need to alter the coordinates to reflect their transformed point.
    double xtrans = 0;
    double ytrans = 0;
    
    wxGraphicsContext* gc = dc->GetGraphicsContext();
    gc->GetTransform().TransformPoint(&xtrans, &ytrans);
    Gdiplus::Graphics* g;
    if (gc) {
        g = (Gdiplus::Graphics*)gc->GetNativeContext();
        hdc = g->GetHDC();
    }
    x += (int)xtrans;
    y += (int)ytrans;    
#else
    hdc = static_cast<HDC>(dc->GetHDC());
#endif

    // ExtTextOut wants the offsets as an array of ints, so extract them
    // from the glyph buffer
    const GlyphBufferGlyph*   glyphs   = glyphBuffer.glyphs(from);
    const GlyphBufferAdvance* advances = glyphBuffer.advances(from);

    int* spacing = new int[numGlyphs - from];
    for (unsigned i = 0; i < numGlyphs; ++i)
        spacing[i] = advances[i].width();

    wxFont* wxfont = font->getWxFont();
    if (wxfont && wxfont->IsOk())
        ::SelectObject(hdc, GetHfontOf(*wxfont));

    if (color.Ok())
        ::SetTextColor(hdc, color.GetPixel());

    // do not draw background behind characters
    ::SetBkMode(hdc, TRANSPARENT);

    // draw text with optional character widths array
    wxString string = wxString((wxChar*)(&glyphs[from]), numGlyphs);
    ::ExtTextOut(hdc, x, y,  ETO_GLYPH_INDEX, 0, reinterpret_cast<const WCHAR*>(glyphs), numGlyphs, spacing);

    ::SetBkMode(hdc, TRANSPARENT);

    #if USE(WXGC)
        g->ReleaseHDC(hdc);
    #endif

    delete [] spacing;
}

}
