

#ifndef FontPlatformData_h
#define FontPlatformData_h

#include "StringImpl.h"

#ifdef __OBJC__
@class NSFont;
#else
class NSFont;
#endif

typedef struct CGFont* CGFontRef;
#ifndef BUILDING_ON_TIGER
typedef const struct __CTFont* CTFontRef;
#endif

#include <CoreFoundation/CFBase.h>
#include <objc/objc-auto.h>
#include <wtf/RetainPtr.h>

typedef UInt32 ATSUFontID;

namespace WebCore {

class String;

#ifndef BUILDING_ON_TIGER
inline CTFontRef toCTFontRef(NSFont *nsFont) { return reinterpret_cast<CTFontRef>(nsFont); }
#endif

struct FontPlatformData {
    FontPlatformData(float size, bool syntheticBold, bool syntheticOblique)
        : m_syntheticBold(syntheticBold)
        , m_syntheticOblique(syntheticOblique)
        , m_atsuFontID(0)
        , m_size(size)
        , m_font(0)
#ifdef BUILDING_ON_TIGER
        , m_cgFont(0)
#endif
    {
    }

    FontPlatformData(NSFont *nsFont, bool syntheticBold = false, bool syntheticOblique = false);
    
    FontPlatformData(CGFontRef cgFont, ATSUFontID fontID, float size, bool syntheticBold, bool syntheticOblique)
        : m_syntheticBold(syntheticBold)
        , m_syntheticOblique(syntheticOblique)
        , m_atsuFontID(fontID)
        , m_size(size)
        , m_font(0)
        , m_cgFont(cgFont)
    {
    }

    FontPlatformData(const FontPlatformData&);
    
    ~FontPlatformData();

    FontPlatformData(WTF::HashTableDeletedValueType) : m_font(hashTableDeletedFontValue()) { }
    bool isHashTableDeletedValue() const { return m_font == hashTableDeletedFontValue(); }

    float size() const { return m_size; }
    bool syntheticBold() const { return m_syntheticBold; }
    bool syntheticOblique() const { return m_syntheticOblique; }

    bool m_syntheticBold;
    bool m_syntheticOblique;

    ATSUFontID m_atsuFontID;
    float m_size;

    unsigned hash() const
    {
        ASSERT(m_font != 0 || m_cgFont == 0);
        uintptr_t hashCodes[2] = { (uintptr_t)m_font, m_syntheticBold << 1 | m_syntheticOblique };
        return StringImpl::computeHash(reinterpret_cast<UChar*>(hashCodes), sizeof(hashCodes) / sizeof(UChar));
    }

    const FontPlatformData& operator=(const FontPlatformData& f);

    bool operator==(const FontPlatformData& other) const
    { 
        return m_font == other.m_font && m_syntheticBold == other.m_syntheticBold && m_syntheticOblique == other.m_syntheticOblique && 
               m_cgFont == other.m_cgFont && m_size == other.m_size && m_atsuFontID == other.m_atsuFontID;
    }

    NSFont *font() const { return m_font; }
    void setFont(NSFont *font);

    bool roundsGlyphAdvances() const;
    bool allowsLigatures() const;

#ifndef BUILDING_ON_TIGER
    CGFontRef cgFont() const { return m_cgFont.get(); }
#else
    CGFontRef cgFont() const { return m_cgFont; }
#endif

#ifndef NDEBUG
    String description() const;
#endif

private:
    static NSFont *hashTableDeletedFontValue() { return reinterpret_cast<NSFont *>(-1); }

    NSFont *m_font;
#ifndef BUILDING_ON_TIGER
    RetainPtr<CGFontRef> m_cgFont;
#else
    CGFontRef m_cgFont; // It is not necessary to refcount this, since either an NSFont owns it or some CachedFont has it referenced.
#endif
};

}

#endif
