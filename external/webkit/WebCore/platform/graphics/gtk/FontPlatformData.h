

#ifndef FontPlatformData_h
#define FontPlatformData_h

#include "GlyphBuffer.h"
#include "FontDescription.h"
#include <cairo.h>
#if defined(USE_FREETYPE)
#include <cairo-ft.h>
#include <fontconfig/fcfreetype.h>
#elif defined(USE_PANGO)
#include <pango/pangocairo.h>
#else
#error "Must defined a font backend"
#endif

namespace WebCore {

class String;

class FontPlatformData {
public:
    FontPlatformData(WTF::HashTableDeletedValueType)
#if defined(USE_FREETYPE)
        : m_pattern(hashTableDeletedFontValue())
        , m_fallbacks(0)
#elif defined(USE_PANGO)
        : m_context(0)
        , m_font(hashTableDeletedFontValue())
#else
#error "Must defined a font backend"
#endif
        , m_scaledFont(0)
        { }

    FontPlatformData()
#if defined(USE_FREETYPE)
        : m_pattern(0)
        , m_fallbacks(0)
#elif defined(USE_PANGO)
        : m_context(0)
        , m_font(0)
#else
#error "Must defined a font backend"
#endif
        , m_scaledFont(0)
        { }

    FontPlatformData(const FontDescription&, const AtomicString& family);

    FontPlatformData(float size, bool bold, bool italic);
    FontPlatformData(cairo_font_face_t* fontFace, int size, bool bold, bool italic);
    FontPlatformData(const FontPlatformData&);

    ~FontPlatformData();

    static bool init();

    bool isFixedPitch();
    float size() const { return m_size; }
    bool syntheticBold() const { return m_syntheticBold; }
    bool syntheticOblique() const { return m_syntheticOblique; }

    cairo_scaled_font_t* scaledFont() const { return m_scaledFont; }

    unsigned hash() const
    {
#if defined(USE_FREETYPE)
        if (m_pattern)
            return FcPatternHash(m_pattern);
#endif
        uintptr_t hashCodes[1] = { reinterpret_cast<uintptr_t>(m_scaledFont) };
        return StringImpl::computeHash(reinterpret_cast<UChar*>(hashCodes), sizeof(hashCodes) / sizeof(UChar));
    }

    bool operator==(const FontPlatformData&) const;
    FontPlatformData& operator=(const FontPlatformData&);
    bool isHashTableDeletedValue() const {
#if defined(USE_FREETYPE)
        return m_pattern == hashTableDeletedFontValue();
#elif defined(USE_PANGO)
        return m_font == hashTableDeletedFontValue();
#endif
    };

#ifndef NDEBUG
    String description() const;
#endif

#if defined(USE_FREETYPE)
    FcPattern* m_pattern;
    FcFontSet* m_fallbacks;
#elif defined(USE_PANGO)
    static PangoFontMap* m_fontMap;
    static GHashTable* m_hashTable;

    PangoContext* m_context;
    PangoFont* m_font;
#else
#error "Must defined a font backend"
#endif
    float m_size;
    bool m_syntheticBold;
    bool m_syntheticOblique;
    cairo_scaled_font_t* m_scaledFont;
private:
#if defined(USE_FREETYPE)
    static FcPattern *hashTableDeletedFontValue() { return reinterpret_cast<FcPattern*>(-1); }
#elif defined(USE_PANGO)
    static PangoFont *hashTableDeletedFontValue() { return reinterpret_cast<PangoFont*>(-1); }
#endif
};

}

#endif
