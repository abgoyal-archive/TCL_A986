

#ifndef FontCache_h
#define FontCache_h

#include <limits.h>
#include <wtf/Vector.h>
#include <wtf/unicode/Unicode.h>

#if PLATFORM(WIN)
#include <objidl.h>
#include <mlang.h>
#endif

namespace WebCore
{

class AtomicString;
class Font;
class FontPlatformData;
class FontData;
class FontDescription;
class FontSelector;
class SimpleFontData;

class FontCache : public Noncopyable {
public:
    friend FontCache* fontCache();

    const FontData* getFontData(const Font&, int& familyIndex, FontSelector*);
    void releaseFontData(const SimpleFontData*);
    
    // This method is implemented by the platform.
    // FIXME: Font data returned by this method never go inactive because callers don't track and release them.
    const SimpleFontData* getFontDataForCharacters(const Font&, const UChar* characters, int length);
    
    // Also implemented by the platform.
    void platformInit();

#if OS(WINCE) && !PLATFORM(QT)
#if defined(IMLANG_FONT_LINK) && (IMLANG_FONT_LINK == 2)
    IMLangFontLink2* getFontLinkInterface();
#else
    IMLangFontLink* getFontLinkInterface();
#endif
    static void comInitialize();
    static void comUninitialize();
#elif PLATFORM(WIN)
    IMLangFontLink2* getFontLinkInterface();
#endif

    void getTraitsInFamily(const AtomicString&, Vector<unsigned>&);

    SimpleFontData* getCachedFontData(const FontDescription& fontDescription, const AtomicString& family, bool checkingAlternateName = false);
    SimpleFontData* getLastResortFallbackFont(const FontDescription&);

    void addClient(FontSelector*);
    void removeClient(FontSelector*);

    unsigned generation();
    void invalidate();

    size_t fontDataCount();
    size_t inactiveFontDataCount();
    void purgeInactiveFontData(int count = INT_MAX);

private:
    FontCache();
    ~FontCache();

    // FIXME: This method should eventually be removed.
    FontPlatformData* getCachedFontPlatformData(const FontDescription&, const AtomicString& family, bool checkingAlternateName = false);

    // These methods are implemented by each platform.
    SimpleFontData* getSimilarFontPlatformData(const Font&);
    FontPlatformData* createFontPlatformData(const FontDescription&, const AtomicString& family);

    SimpleFontData* getCachedFontData(const FontPlatformData*);

    friend class SimpleFontData; // For getCachedFontData(const FontPlatformData*)
    friend class FontFallbackList;
};

// Get the global fontCache.
FontCache* fontCache();

}

#endif
