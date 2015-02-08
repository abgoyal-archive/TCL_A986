

#ifndef WidthIterator_h
#define WidthIterator_h

#include <wtf/HashSet.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

class Font;
class GlyphBuffer;
class SimpleFontData;
class TextRun;

struct WidthIterator {
    WidthIterator(const Font*, const TextRun&, HashSet<const SimpleFontData*>* fallbackFonts = 0);

    void advance(int to, GlyphBuffer* = 0);
    bool advanceOneCharacter(float& width, GlyphBuffer* = 0);

    const Font* m_font;

    const TextRun& m_run;
    int m_end;

    unsigned m_currentCharacter;
    float m_runWidthSoFar;
    float m_padding;
    float m_padPerSpace;
    float m_finalRoundingWidth;

private:
    UChar32 normalizeVoicingMarks(int currentCharacter);
    HashSet<const SimpleFontData*>* m_fallbackFonts;
};

}

#endif
