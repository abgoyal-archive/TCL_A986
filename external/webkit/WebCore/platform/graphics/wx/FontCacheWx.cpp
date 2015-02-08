

#include "config.h"
#include "FontCache.h"
#include "Font.h"
#include "FontPlatformData.h"
#include "NotImplemented.h"
#include "SimpleFontData.h"

namespace WebCore {

void FontCache::platformInit()
{
}

const SimpleFontData* FontCache::getFontDataForCharacters(const Font& font, const UChar* characters, int length)
{
    SimpleFontData* fontData = 0;
    fontData = new SimpleFontData(FontPlatformData(font.fontDescription(), font.family().family()));
    return fontData;
}

SimpleFontData* FontCache::getSimilarFontPlatformData(const Font& font)
{
    return getCachedFontData(font.fontDescription(), font.family().family());
}

SimpleFontData* FontCache::getLastResortFallbackFont(const FontDescription& fontDescription)
{
    // FIXME: Would be even better to somehow get the user's default font here.  For now we'll pick
    // the default that the user would get without changing any prefs.
    static AtomicString timesStr("systemfont");
    return getCachedFontData(fontDescription, timesStr);
}

FontPlatformData* FontCache::createFontPlatformData(const FontDescription& fontDescription, const AtomicString& family)
{
    return new FontPlatformData(fontDescription,family);
}

void FontCache::getTraitsInFamily(const AtomicString& familyName, Vector<unsigned>& traitsMasks)
{
    notImplemented();
}

}
