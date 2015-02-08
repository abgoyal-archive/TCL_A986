

#include "config.h"
#include "CSSFontFace.h"

#include "CSSFontFaceSource.h"
#include "CSSFontSelector.h"
#include "CSSSegmentedFontFace.h"
#include "FontDescription.h"
#include "SimpleFontData.h"

namespace WebCore {

CSSFontFace::~CSSFontFace()
{
    deleteAllValues(m_sources);
}

bool CSSFontFace::isLoaded() const
{
    unsigned size = m_sources.size();
    for (unsigned i = 0; i < size; i++) {
        if (!m_sources[i]->isLoaded())
            return false;
    }
    return true;
}

bool CSSFontFace::isValid() const
{
    unsigned size = m_sources.size();
    if (!size)
        return false;
    for (unsigned i = 0; i < size; i++) {
        if (m_sources[i]->isValid())
            return true;
    }
    return false;
}

void CSSFontFace::addedToSegmentedFontFace(CSSSegmentedFontFace* segmentedFontFace)
{
    m_segmentedFontFaces.add(segmentedFontFace);
}

void CSSFontFace::removedFromSegmentedFontFace(CSSSegmentedFontFace* segmentedFontFace)
{
    m_segmentedFontFaces.remove(segmentedFontFace);
}

void CSSFontFace::addSource(CSSFontFaceSource* source)
{
    m_sources.append(source);
    source->setFontFace(this);
}

void CSSFontFace::fontLoaded(CSSFontFaceSource* source)
{
    if (source != m_activeSource)
        return;

    // FIXME: Can we assert that m_segmentedFontFaces is not empty? That may
    // require stopping in-progress font loading when the last
    // CSSSegmentedFontFace is removed.
    if (m_segmentedFontFaces.isEmpty())
        return;

    HashSet<CSSSegmentedFontFace*>::iterator end = m_segmentedFontFaces.end();
    for (HashSet<CSSSegmentedFontFace*>::iterator it = m_segmentedFontFaces.begin(); it != end; ++it)
        (*it)->fontLoaded(this);

    // Use one of the CSSSegmentedFontFaces' font selector. They all have
    // the same font selector, so it's wasteful to store it in the CSSFontFace.
    CSSFontSelector* fontSelector = (*m_segmentedFontFaces.begin())->fontSelector();
    fontSelector->fontLoaded();
}

SimpleFontData* CSSFontFace::getFontData(const FontDescription& fontDescription, bool syntheticBold, bool syntheticItalic)
{
    m_activeSource = 0;
    if (!isValid())
        return 0;

    ASSERT(!m_segmentedFontFaces.isEmpty());
    CSSFontSelector* fontSelector = (*m_segmentedFontFaces.begin())->fontSelector();

    size_t size = m_sources.size();
    for (size_t i = 0; i < size; ++i) {
        if (SimpleFontData* result = m_sources[i]->getFontData(fontDescription, syntheticBold, syntheticItalic, fontSelector)) {
            m_activeSource = m_sources[i];
            return result;
        }
    }

    return 0;
}

}

