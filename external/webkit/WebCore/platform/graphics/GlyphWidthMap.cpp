

#include "config.h"
#include "GlyphWidthMap.h"

namespace WebCore {

GlyphWidthMap::GlyphWidthPage* GlyphWidthMap::locatePageSlowCase(unsigned pageNumber)
{
    GlyphWidthPage* page;
    if (pageNumber == 0) {
        ASSERT(!m_filledPrimaryPage);
        page = &m_primaryPage;
        m_filledPrimaryPage = true;
    } else {
        if (m_pages) {
            if ((page = m_pages->get(pageNumber)))
                return page;
        } else
            m_pages.set(new HashMap<int, GlyphWidthPage*>);
        page = new GlyphWidthPage;
        m_pages->set(pageNumber, page);
    }

    // Fill in the whole page with the unknown glyph width value.
    for (unsigned i = 0; i < GlyphWidthPage::size; i++)
        page->setWidthForIndex(i, cGlyphWidthUnknown);

    return page;
}

}
