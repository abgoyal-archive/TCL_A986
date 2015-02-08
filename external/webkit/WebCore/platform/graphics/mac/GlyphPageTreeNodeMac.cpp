

#include "config.h"
#include "GlyphPageTreeNode.h"

#include "SimpleFontData.h"
#include "WebCoreSystemInterface.h"
#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

bool GlyphPage::fill(unsigned offset, unsigned length, UChar* buffer, unsigned bufferLength, const SimpleFontData* fontData)
{
    bool haveGlyphs = false;

#ifndef BUILDING_ON_TIGER
    Vector<CGGlyph, 512> glyphs(bufferLength);
    wkGetGlyphsForCharacters(fontData->platformData().cgFont(), buffer, glyphs.data(), bufferLength);

    for (unsigned i = 0; i < length; ++i) {
        if (!glyphs[i])
            setGlyphDataForIndex(offset + i, 0, 0);
        else {
            setGlyphDataForIndex(offset + i, glyphs[i], fontData);
            haveGlyphs = true;
        }
    }
#else
    // Use an array of long so we get good enough alignment.
    long glyphVector[(GLYPH_VECTOR_SIZE + sizeof(long) - 1) / sizeof(long)];
    
    OSStatus status = wkInitializeGlyphVector(GlyphPage::size, &glyphVector);
    if (status != noErr)
        // This should never happen, perhaps indicates a bad font!  If it does the
        // font substitution code will find an alternate font.
        return false;

    wkConvertCharToGlyphs(fontData->m_styleGroup, buffer, bufferLength, &glyphVector);

    unsigned numGlyphs = wkGetGlyphVectorNumGlyphs(&glyphVector);
    if (numGlyphs != length) {
        // This should never happen, perhaps indicates a bad font?
        // If it does happen, the font substitution code will find an alternate font.
        wkClearGlyphVector(&glyphVector);
        return false;
    }

    ATSLayoutRecord* glyphRecord = (ATSLayoutRecord*)wkGetGlyphVectorFirstRecord(glyphVector);
    for (unsigned i = 0; i < length; i++) {
        Glyph glyph = glyphRecord->glyphID;
        if (!glyph)
            setGlyphDataForIndex(offset + i, 0, 0);
        else {
            setGlyphDataForIndex(offset + i, glyph, fontData);
            haveGlyphs = true;
        }
        glyphRecord = (ATSLayoutRecord *)((char *)glyphRecord + wkGetGlyphVectorRecordSize(glyphVector));
    }
    wkClearGlyphVector(&glyphVector);
#endif

    return haveGlyphs;
}

} // namespace WebCore
