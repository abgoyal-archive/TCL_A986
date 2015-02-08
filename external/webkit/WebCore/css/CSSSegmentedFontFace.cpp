

#include "config.h"
#include "CSSSegmentedFontFace.h"

#include "CSSFontFace.h"
#include "CSSFontSelector.h"
#include "FontDescription.h"
#include "SegmentedFontData.h"
#include "SimpleFontData.h"

namespace WebCore {

CSSSegmentedFontFace::CSSSegmentedFontFace(CSSFontSelector* fontSelector)
    : m_fontSelector(fontSelector)
{
}

CSSSegmentedFontFace::~CSSSegmentedFontFace()
{
    pruneTable();
    unsigned size = m_fontFaces.size();
    for (unsigned i = 0; i < size; i++)
        m_fontFaces[i]->removedFromSegmentedFontFace(this);
}

void CSSSegmentedFontFace::pruneTable()
{
    // Make sure the glyph page tree prunes out all uses of this custom font.
    if (m_fontDataTable.isEmpty())
        return;
    HashMap<unsigned, SegmentedFontData*>::iterator end = m_fontDataTable.end();
    for (HashMap<unsigned, SegmentedFontData*>::iterator it = m_fontDataTable.begin(); it != end; ++it)
        GlyphPageTreeNode::pruneTreeCustomFontData(it->second);
    deleteAllValues(m_fontDataTable);
    m_fontDataTable.clear();
}

bool CSSSegmentedFontFace::isLoaded() const
{
    unsigned size = m_fontFaces.size();
    for (unsigned i = 0; i < size; i++) {
        if (!m_fontFaces[i]->isLoaded())
            return false;
    }
    return true;
}

bool CSSSegmentedFontFace::isValid() const
{
    unsigned size = m_fontFaces.size();
    for (unsigned i = 0; i < size; i++) {
        if (!m_fontFaces[i]->isValid())
            return false;
    }
    return true;
}

void CSSSegmentedFontFace::fontLoaded(CSSFontFace*)
{
    pruneTable();
}

void CSSSegmentedFontFace::appendFontFace(PassRefPtr<CSSFontFace> fontFace)
{
    pruneTable();
    fontFace->addedToSegmentedFontFace(this);
    m_fontFaces.append(fontFace);
}

FontData* CSSSegmentedFontFace::getFontData(const FontDescription& fontDescription)
{
    if (!isValid())
        return 0;

    FontTraitsMask desiredTraitsMask = fontDescription.traitsMask();
    unsigned hashKey = fontDescription.computedPixelSize() << FontTraitsMaskWidth | desiredTraitsMask;

    SegmentedFontData* fontData = m_fontDataTable.get(hashKey);
    if (fontData)
        return fontData;

    fontData = new SegmentedFontData();

    unsigned size = m_fontFaces.size();
    for (unsigned i = 0; i < size; i++) {
        FontTraitsMask traitsMask = m_fontFaces[i]->traitsMask();
        bool syntheticBold = !(traitsMask & (FontWeight600Mask | FontWeight700Mask | FontWeight800Mask | FontWeight900Mask)) && (desiredTraitsMask & (FontWeight600Mask | FontWeight700Mask | FontWeight800Mask | FontWeight900Mask));
        bool syntheticItalic = !(traitsMask & FontStyleItalicMask) && (desiredTraitsMask & FontStyleItalicMask);
        if (const FontData* faceFontData = m_fontFaces[i]->getFontData(fontDescription, syntheticBold, syntheticItalic)) {
            ASSERT(!faceFontData->isSegmented());
            const Vector<CSSFontFace::UnicodeRange>& ranges = m_fontFaces[i]->ranges();
            unsigned numRanges = ranges.size();
            if (!numRanges)
                fontData->appendRange(FontDataRange(0, 0x7FFFFFFF, static_cast<const SimpleFontData*>(faceFontData)));
            else {
                for (unsigned j = 0; j < numRanges; ++j)
                    fontData->appendRange(FontDataRange(ranges[j].from(), ranges[j].to(), static_cast<const SimpleFontData*>(faceFontData)));
            }
        }
    }
    if (fontData->numRanges())
        m_fontDataTable.set(hashKey, fontData);
    else {
        delete fontData;
        fontData = 0;
    }

    return fontData;
}

}
