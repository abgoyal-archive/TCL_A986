

#ifndef SegmentedFontData_h
#define SegmentedFontData_h

#include "FontData.h"
#include <wtf/Vector.h>

namespace WebCore {

class SimpleFontData;

struct FontDataRange {
    FontDataRange(UChar32 from, UChar32 to, const SimpleFontData* fontData)
        : m_from(from)
        , m_to(to)
        , m_fontData(fontData)
    {
    }

    UChar32 from() const { return m_from; }
    UChar32 to() const { return m_to; }
    const SimpleFontData* fontData() const { return m_fontData; }

private:
    UChar32 m_from;
    UChar32 m_to;
    const SimpleFontData* m_fontData;
};

class SegmentedFontData : public FontData {
public:
    virtual ~SegmentedFontData();

    void appendRange(const FontDataRange& range) { m_ranges.append(range); }
    unsigned numRanges() const { return m_ranges.size(); }
    const FontDataRange& rangeAt(unsigned i) const { return m_ranges[i]; }

#ifndef NDEBUG
    virtual String description() const;
#endif

private:
    virtual const SimpleFontData* fontDataForCharacter(UChar32) const;
    virtual bool containsCharacters(const UChar*, int length) const;

    virtual bool isCustomFont() const;
    virtual bool isLoading() const;
    virtual bool isSegmented() const;

    bool containsCharacter(UChar32) const;

    Vector<FontDataRange, 1> m_ranges;
};

} // namespace WebCore

#endif // SegmentedFontData_h
