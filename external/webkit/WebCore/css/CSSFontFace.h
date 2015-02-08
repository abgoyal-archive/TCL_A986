

#ifndef CSSFontFace_h
#define CSSFontFace_h

#include "FontTraitsMask.h"
#include <wtf/HashSet.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

class CSSFontFaceSource;
class CSSSegmentedFontFace;
class FontDescription;
class SimpleFontData;

class CSSFontFace : public RefCounted<CSSFontFace> {
public:
    static PassRefPtr<CSSFontFace> create(FontTraitsMask traitsMask) { return adoptRef(new CSSFontFace(traitsMask)); }
    ~CSSFontFace();

    FontTraitsMask traitsMask() const { return m_traitsMask; }

    struct UnicodeRange;

    void addRange(UChar32 from, UChar32 to) { m_ranges.append(UnicodeRange(from, to)); }
    const Vector<UnicodeRange>& ranges() const { return m_ranges; }

    void addedToSegmentedFontFace(CSSSegmentedFontFace*);
    void removedFromSegmentedFontFace(CSSSegmentedFontFace*);

    bool isLoaded() const;
    bool isValid() const;

    void addSource(CSSFontFaceSource*);

    void fontLoaded(CSSFontFaceSource*);

    SimpleFontData* getFontData(const FontDescription&, bool syntheticBold, bool syntheticItalic);

    struct UnicodeRange {
        UnicodeRange(UChar32 from, UChar32 to)
            : m_from(from)
            , m_to(to)
        {
        }

        UChar32 from() const { return m_from; }
        UChar32 to() const { return m_to; }

    private:
        UChar32 m_from;
        UChar32 m_to;
    };

private:
    CSSFontFace(FontTraitsMask traitsMask)
        : m_traitsMask(traitsMask)
        , m_activeSource(0)
    {
    }

    FontTraitsMask m_traitsMask;
    Vector<UnicodeRange> m_ranges;
    HashSet<CSSSegmentedFontFace*> m_segmentedFontFaces;
    Vector<CSSFontFaceSource*> m_sources;
    CSSFontFaceSource* m_activeSource;
};

}

#endif
