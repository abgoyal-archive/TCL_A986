

#ifndef GlyphWidthMap_h
#define GlyphWidthMap_h

#include <wtf/HashMap.h>
#include <wtf/OwnPtr.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

typedef unsigned short Glyph;

const float cGlyphWidthUnknown = -1;

class GlyphWidthMap : public Noncopyable {
public:
    GlyphWidthMap() : m_filledPrimaryPage(false) { }
    ~GlyphWidthMap() { if (m_pages) { deleteAllValues(*m_pages); } }

    float widthForGlyph(Glyph glyph)
    {
        return locatePage(glyph / GlyphWidthPage::size)->widthForGlyph(glyph);
    }

    void setWidthForGlyph(Glyph glyph, float width)
    {
        locatePage(glyph / GlyphWidthPage::size)->setWidthForGlyph(glyph, width);
    }

private:
    struct GlyphWidthPage {
        static const size_t size = 256; // Usually covers Latin-1 in a single page.
        float m_widths[size];

        float widthForGlyph(Glyph glyph) const { return m_widths[glyph % size]; }
        void setWidthForGlyph(Glyph glyph, float width)
        {
            setWidthForIndex(glyph % size, width);
        }
        void setWidthForIndex(unsigned index, float width)
        {
            m_widths[index] = width;
        }
    };
    
    GlyphWidthPage* locatePage(unsigned pageNumber)
    {
        if (!pageNumber && m_filledPrimaryPage)
            return &m_primaryPage;
        return locatePageSlowCase(pageNumber);
    }

    GlyphWidthPage* locatePageSlowCase(unsigned pageNumber);
    
    bool m_filledPrimaryPage;
    GlyphWidthPage m_primaryPage; // We optimize for the page that contains glyph indices 0-255.
    OwnPtr<HashMap<int, GlyphWidthPage*> > m_pages;
};

}

#endif
