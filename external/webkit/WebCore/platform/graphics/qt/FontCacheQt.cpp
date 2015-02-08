
#include "config.h"
#include "FontCache.h"

#include "FontDescription.h"
#include "FontPlatformData.h"
#include "Font.h"
#include "PlatformString.h"
#include "StringHash.h"
#include <utility>
#include <wtf/ListHashSet.h>
#include <wtf/StdLibExtras.h>

#include <QFont>

using namespace WTF;

namespace WebCore {

void FontCache::platformInit()
{
}

const SimpleFontData* FontCache::getFontDataForCharacters(const Font&, const UChar*, int)
{
    return 0;
}

SimpleFontData* FontCache::getSimilarFontPlatformData(const Font& font)
{
    return 0;
}

SimpleFontData* FontCache::getLastResortFallbackFont(const FontDescription& fontDescription)
{
    const AtomicString fallbackFamily = QFont(fontDescription.family().family()).lastResortFamily();
    return getCachedFontData(new FontPlatformData(fontDescription, fallbackFamily));
}

void FontCache::getTraitsInFamily(const AtomicString&, Vector<unsigned>&)
{
}

FontPlatformData* FontCache::createFontPlatformData(const FontDescription& fontDescription, const AtomicString& familyName)
{
    return new FontPlatformData(fontDescription, familyName);
}

} // namespace WebCore
