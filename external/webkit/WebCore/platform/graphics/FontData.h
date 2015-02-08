

#ifndef FontData_h
#define FontData_h

#include <wtf/Noncopyable.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

class SimpleFontData;
class String;

class FontData : public Noncopyable {
public:
    FontData()
        : m_maxGlyphPageTreeLevel(0)
    {
    }

    virtual ~FontData();

    virtual const SimpleFontData* fontDataForCharacter(UChar32) const = 0;
    virtual bool containsCharacters(const UChar*, int length) const = 0;
    virtual bool isCustomFont() const = 0;
    virtual bool isLoading() const = 0;
    virtual bool isSegmented() const = 0;

    void setMaxGlyphPageTreeLevel(unsigned level) const { m_maxGlyphPageTreeLevel = level; }
    unsigned maxGlyphPageTreeLevel() const { return m_maxGlyphPageTreeLevel; }

#ifndef NDEBUG
    virtual String description() const = 0;
#endif

private:
    mutable unsigned m_maxGlyphPageTreeLevel;
};

} // namespace WebCore

#endif // FontData_h
