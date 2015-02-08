

#ifndef FontPlatformDataWince_H
#define FontPlatformDataWince_H

#include "FontDescription.h"
#include "StringImpl.h"
#include <wtf/Noncopyable.h>

typedef struct tagTEXTMETRICW TEXTMETRIC;
typedef struct tagLOGFONTW LOGFONT;

namespace WebCore {

    class FontPlatformPrivateData;
    class String;

    class FontPlatformData {

    public:

        FontPlatformData(): m_private(0) {}
        FontPlatformData(float size, bool bold, bool oblique);

        // Used for deleted values in the font cache's hash tables.
        FontPlatformData(WTF::HashTableDeletedValueType) : m_private((FontPlatformPrivateData*)1) {}
        bool isHashTableDeletedValue() const { return (unsigned)m_private == 1; }

        FontPlatformData(const FontDescription& fontDescription, const AtomicString& family, bool useDefaultFontIfNotPresent = true);

        ~FontPlatformData();

        FontPlatformData(const FontPlatformData& o) : m_private(0) { operator=(o); }
        FontPlatformData& operator=(const FontPlatformData& o);

        int isValid() const { return reinterpret_cast<unsigned>(m_private) & ~1; }
        HFONT hfont() const;
        const TEXTMETRIC& metrics() const;
        bool isSystemFont() const;
        int size() const;
        unsigned hash() const { return (unsigned)m_private; }
        const FontDescription& fontDescription() const;
        const AtomicString& family() const;
        bool operator==(const FontPlatformData& other) const {     return m_private == other.m_private; }
        HFONT getScaledFontHandle(int height, int width) const;
        const LOGFONT& logFont() const;
        int averageCharWidth() const;
        bool isDisabled() const;
        bool discardFontHandle();
        DWORD codePages() const;

        static bool isSongTiSupported();
        static bool mapKnownFont(DWORD codePages, String& family);
        static DWORD getKnownFontCodePages(const wchar_t* family);
        static const String& defaultFontFamily();
        static LONG adjustedGDIFontWeight(LONG gdiFontWeight, const String& family);

#ifndef NDEBUG
        String description() const;
#endif

    private:
        FontPlatformPrivateData* m_private;
    };

}

#endif
