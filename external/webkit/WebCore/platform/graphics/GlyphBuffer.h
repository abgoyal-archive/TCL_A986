

#ifndef GlyphBuffer_h
#define GlyphBuffer_h

#include "FloatSize.h"
#include <wtf/UnusedParam.h>
#include <wtf/Vector.h>

#if PLATFORM(CG)
#include <ApplicationServices/ApplicationServices.h>
#endif

#if PLATFORM(CAIRO) || (PLATFORM(WX) && defined(__WXGTK__))
#include <cairo.h>
#endif

namespace WebCore {

typedef unsigned short Glyph;
class SimpleFontData;

#if PLATFORM(CAIRO)
// FIXME: Why does Cairo use such a huge struct instead of just an offset into an array?
typedef cairo_glyph_t GlyphBufferGlyph;
#elif OS(WINCE)
typedef wchar_t GlyphBufferGlyph;
#else
typedef Glyph GlyphBufferGlyph;
#endif

// CG uses CGSize instead of FloatSize so that the result of advances()
// can be passed directly to CGContextShowGlyphsWithAdvances in FontMac.mm
#if PLATFORM(CG)
typedef CGSize GlyphBufferAdvance;
#elif OS(WINCE)
// There is no cross-platform code that uses the height of GlyphBufferAdvance,
// so we can save memory space on embedded devices by storing only the width
typedef float GlyphBufferAdvance;
#else
typedef FloatSize GlyphBufferAdvance;
#endif

class GlyphBuffer {
public:
    bool isEmpty() const { return m_fontData.isEmpty(); }
    int size() const { return m_fontData.size(); }
    
    void clear()
    {
        m_fontData.clear();
        m_glyphs.clear();
        m_advances.clear();
#if PLATFORM(WIN)
        m_offsets.clear();
#endif
    }

    GlyphBufferGlyph* glyphs(int from) { return m_glyphs.data() + from; }
    GlyphBufferAdvance* advances(int from) { return m_advances.data() + from; }
    const GlyphBufferGlyph* glyphs(int from) const { return m_glyphs.data() + from; }
    const GlyphBufferAdvance* advances(int from) const { return m_advances.data() + from; }

    const SimpleFontData* fontDataAt(int index) const { return m_fontData[index]; }
    
    void swap(int index1, int index2)
    {
        const SimpleFontData* f = m_fontData[index1];
        m_fontData[index1] = m_fontData[index2];
        m_fontData[index2] = f;

        GlyphBufferGlyph g = m_glyphs[index1];
        m_glyphs[index1] = m_glyphs[index2];
        m_glyphs[index2] = g;

        GlyphBufferAdvance s = m_advances[index1];
        m_advances[index1] = m_advances[index2];
        m_advances[index2] = s;

#if PLATFORM(WIN)
        FloatSize offset = m_offsets[index1];
        m_offsets[index1] = m_offsets[index2];
        m_offsets[index2] = offset;
#endif
    }

    Glyph glyphAt(int index) const
    {
#if PLATFORM(CAIRO)
        return m_glyphs[index].index;
#else
        return m_glyphs[index];
#endif
    }

    float advanceAt(int index) const
    {
#if PLATFORM(CG)
        return m_advances[index].width;
#elif OS(WINCE)
        return m_advances[index];
#else
        return m_advances[index].width();
#endif
    }

    FloatSize offsetAt(int index) const
    {
#if PLATFORM(WIN)
        return m_offsets[index];
#else
        UNUSED_PARAM(index);
        return FloatSize();
#endif
    }

    void add(Glyph glyph, const SimpleFontData* font, float width, const FloatSize* offset = 0)
    {
        m_fontData.append(font);

#if PLATFORM(CAIRO)
        cairo_glyph_t cairoGlyph;
        cairoGlyph.index = glyph;
        m_glyphs.append(cairoGlyph);
#else
        m_glyphs.append(glyph);
#endif

#if PLATFORM(CG)
        CGSize advance = { width, 0 };
        m_advances.append(advance);
#elif OS(WINCE)
        m_advances.append(width);
#else
        m_advances.append(FloatSize(width, 0));
#endif

#if PLATFORM(WIN)
        if (offset)
            m_offsets.append(*offset);
        else
            m_offsets.append(FloatSize());
#else
        UNUSED_PARAM(offset);
#endif
    }
    
#if !OS(WINCE)
    void add(Glyph glyph, const SimpleFontData* font, GlyphBufferAdvance advance)
    {
        m_fontData.append(font);
#if PLATFORM(CAIRO)
        cairo_glyph_t cairoGlyph;
        cairoGlyph.index = glyph;
        m_glyphs.append(cairoGlyph);
#else
        m_glyphs.append(glyph);
#endif

        m_advances.append(advance);
    }
#endif
    
private:
    Vector<const SimpleFontData*, 2048> m_fontData;
    Vector<GlyphBufferGlyph, 2048> m_glyphs;
    Vector<GlyphBufferAdvance, 2048> m_advances;
#if PLATFORM(WIN)
    Vector<FloatSize, 2048> m_offsets;
#endif
};

}
#endif
