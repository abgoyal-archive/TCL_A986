

#include "config.h"
#include "FontCache.h"

#include "AtomicString.h"
#include "ChromiumBridge.h"
#include "CString.h"
#include "Font.h"
#include "FontDescription.h"
#include "FontPlatformData.h"
#include "Logging.h"
#include "NotImplemented.h"
#include "SimpleFontData.h"

#include "SkPaint.h"
#include "SkTypeface.h"
#include "SkUtils.h"

#include <unicode/utf16.h>
#include <wtf/Assertions.h>

namespace WebCore {

void FontCache::platformInit()
{
}

const SimpleFontData* FontCache::getFontDataForCharacters(const Font& font,
                                                          const UChar* characters,
                                                          int length)
{
    String family = ChromiumBridge::getFontFamilyForCharacters(characters, length);
    if (family.isEmpty())
        return 0;

    AtomicString atomicFamily(family);
    return getCachedFontData(getCachedFontPlatformData(font.fontDescription(), atomicFamily, false));
}

SimpleFontData* FontCache::getSimilarFontPlatformData(const Font& font)
{
    return 0;
}

SimpleFontData* FontCache::getLastResortFallbackFont(const FontDescription& description)
{
    static const AtomicString sansStr("Sans");
    static const AtomicString serifStr("Serif");
    static const AtomicString monospaceStr("Monospace");

    FontPlatformData* fontPlatformData = 0;
    switch (description.genericFamily()) {
    case FontDescription::SerifFamily:
        fontPlatformData = getCachedFontPlatformData(description, serifStr);
        break;
    case FontDescription::MonospaceFamily:
        fontPlatformData = getCachedFontPlatformData(description, monospaceStr);
        break;
    case FontDescription::SansSerifFamily:
    default:
        fontPlatformData = getCachedFontPlatformData(description, sansStr);
        break;
    }

    ASSERT(fontPlatformData);
    return getCachedFontData(fontPlatformData);
}

void FontCache::getTraitsInFamily(const AtomicString& familyName,
                                  Vector<unsigned>& traitsMasks)
{
    notImplemented();
}

FontPlatformData* FontCache::createFontPlatformData(const FontDescription& fontDescription,
                                                    const AtomicString& family)
{
    const char* name = 0;
    CString s;

    if (family.length() == 0) {
        static const struct {
            FontDescription::GenericFamilyType mType;
            const char* mName;
        } fontDescriptions[] = {
            { FontDescription::SerifFamily, "serif" },
            { FontDescription::SansSerifFamily, "sans-serif" },
            { FontDescription::MonospaceFamily, "monospace" },
            { FontDescription::CursiveFamily, "cursive" },
            { FontDescription::FantasyFamily, "fantasy" }
        };

        FontDescription::GenericFamilyType type = fontDescription.genericFamily();
        for (unsigned i = 0; i < SK_ARRAY_COUNT(fontDescriptions); i++) {
            if (type == fontDescriptions[i].mType) {
                name = fontDescriptions[i].mName;
                break;
            }
        }
        // if we fall out of the loop, it's ok for name to still be 0
    }
    else {    // convert the name to utf8
        s = family.string().utf8();
        name = s.data();
    }

    int style = SkTypeface::kNormal;
    if (fontDescription.weight() >= FontWeightBold)
        style |= SkTypeface::kBold;
    if (fontDescription.italic())
        style |= SkTypeface::kItalic;

    SkTypeface* tf = SkTypeface::CreateFromName(name, static_cast<SkTypeface::Style>(style));
    if (!tf)
        return 0;

    FontPlatformData* result =
        new FontPlatformData(tf,
                             fontDescription.computedSize(),
                             (style & SkTypeface::kBold) && !tf->isBold(),
                             (style & SkTypeface::kItalic) && !tf->isItalic());
    tf->unref();
    return result;
}

}  // namespace WebCore
