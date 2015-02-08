

#include "config.h"
#include "FontCache.h"

#include "Font.h"
#include "FontData.h"
#include "FontPlatformData.h"
#include "NotImplemented.h"
#include <String.h>


namespace WebCore {

void FontCache::platformInit()
{
}

const SimpleFontData* FontCache::getFontDataForCharacters(const Font& font, const UChar* characters, int length)
{
    FontPlatformData data(font.fontDescription(), font.family().family());
    return getCachedFontData(&data);
}

SimpleFontData* FontCache::getSimilarFontPlatformData(const Font& font)
{
    notImplemented();
    return 0;
}

SimpleFontData* FontCache::getLastResortFallbackFont(const FontDescription& fontDescription)
{
    // FIXME: Would be even better to somehow get the user's default font here.
    // For now we'll pick the default that the user would get without changing any prefs.
    static AtomicString defaultString("DejaVu Serif");
    return getCachedFontData(fontDescription, defaultString);
}

FontPlatformData* FontCache::createFontPlatformData(const FontDescription& fontDescription, const AtomicString& family)
{
    return new FontPlatformData(fontDescription, family);
}

void FontCache::getTraitsInFamily(const AtomicString& familyName, Vector<unsigned>& traitsMasks)
{
    notImplemented();
}

} // namespace WebCore

