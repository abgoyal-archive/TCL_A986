

#ifndef CSSSegmentedFontFace_h
#define CSSSegmentedFontFace_h

#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

class CSSFontFace;
class CSSFontSelector;
class FontData;
class FontDescription;
class SegmentedFontData;

class CSSSegmentedFontFace : public RefCounted<CSSSegmentedFontFace> {
public:
    static PassRefPtr<CSSSegmentedFontFace> create(CSSFontSelector* selector) { return adoptRef(new CSSSegmentedFontFace(selector)); }
    ~CSSSegmentedFontFace();

    bool isLoaded() const;
    bool isValid() const;
    CSSFontSelector* fontSelector() const { return m_fontSelector; }

    void fontLoaded(CSSFontFace*);

    void appendFontFace(PassRefPtr<CSSFontFace>);

    FontData* getFontData(const FontDescription&);

private:
    CSSSegmentedFontFace(CSSFontSelector*);

    void pruneTable();

    CSSFontSelector* m_fontSelector;
    HashMap<unsigned, SegmentedFontData*> m_fontDataTable;
    Vector<RefPtr<CSSFontFace>, 1> m_fontFaces;
};

} // namespace WebCore

#endif // CSSSegmentedFontFace_h
