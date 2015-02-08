

#include "config.h"
#include "SegmentedFontData.h"

#include "PlatformString.h"
#include "SimpleFontData.h"
#include <wtf/Assertions.h>

namespace WebCore {

SegmentedFontData::~SegmentedFontData()
{
}

const SimpleFontData* SegmentedFontData::fontDataForCharacter(UChar32 c) const
{
    Vector<FontDataRange>::const_iterator end = m_ranges.end();
    for (Vector<FontDataRange>::const_iterator it = m_ranges.begin(); it != end; ++it) {
        if (it->from() <= c && it->to() >= c)
            return it->fontData();
    }
    return m_ranges[0].fontData();
}

bool SegmentedFontData::containsCharacter(UChar32 c) const
{
    Vector<FontDataRange>::const_iterator end = m_ranges.end();
    for (Vector<FontDataRange>::const_iterator it = m_ranges.begin(); it != end; ++it) {
        if (c >= it->from() && c <= it->to())
            return true;
    }
    return false;
}

bool SegmentedFontData::containsCharacters(const UChar* characters, int length) const
{
    UChar32 c;
    for (int i = 0; i < length; ) {
        U16_NEXT(characters, i, length, c)
        if (!containsCharacter(c))
            return false;
    }
    return true;
}

bool SegmentedFontData::isCustomFont() const
{
    // All segmented fonts are custom fonts.
    return true;
}

bool SegmentedFontData::isLoading() const
{
    Vector<FontDataRange>::const_iterator end = m_ranges.end();
    for (Vector<FontDataRange>::const_iterator it = m_ranges.begin(); it != end; ++it) {
        if (it->fontData()->isLoading())
            return true;
    }
    return false;
}

bool SegmentedFontData::isSegmented() const
{
    return true;
}

#ifndef NDEBUG
String SegmentedFontData::description() const
{
    return "[segmented font]";
}
#endif

} // namespace WebCore
