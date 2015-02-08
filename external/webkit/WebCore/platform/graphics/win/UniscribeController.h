

#ifndef UniscribeController_h
#define UniscribeController_h

#include <usp10.h>
#include "Font.h"
#include "GlyphBuffer.h"
#include "Vector.h"

namespace WebCore {

class UniscribeController {
public:
    UniscribeController(const Font*, const TextRun&, HashSet<const SimpleFontData*>* fallbackFonts = 0);

    // Advance and measure/place up to the specified character.
    void advance(unsigned to, GlyphBuffer* = 0);

    // Compute the character offset for a given x coordinate.
    int offsetForPosition(int x, bool includePartialGlyphs);

    // Returns the width of everything we've consumed so far.
    float runWidthSoFar() const { return m_runWidthSoFar; }

private:    
    void resetControlAndState();

    void itemizeShapeAndPlace(const UChar*, unsigned length, const SimpleFontData*, GlyphBuffer*);
    bool shapeAndPlaceItem(const UChar*, unsigned index, const SimpleFontData*, GlyphBuffer*);
    bool shape(const UChar* str, int len, SCRIPT_ITEM item, const SimpleFontData* fontData,
               Vector<WORD>& glyphs, Vector<WORD>& clusters,
               Vector<SCRIPT_VISATTR>& visualAttributes);

    const Font& m_font;
    const TextRun& m_run;
    HashSet<const SimpleFontData*>* m_fallbackFonts;

    SCRIPT_CONTROL m_control;
    SCRIPT_STATE m_state;
    Vector<SCRIPT_ITEM> m_items;
 
    unsigned m_currentCharacter;
    int m_end;

    float m_runWidthSoFar;
    float m_padding;
    float m_padPerSpace;

    bool m_computingOffsetPosition;
    bool m_includePartialGlyphs;
    float m_offsetX;
    int m_offsetPosition;
};

}
#endif
