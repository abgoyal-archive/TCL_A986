

#ifndef FontPlatformData_H
#define FontPlatformData_H

#include "FontDescription.h"
#include "GlyphBuffer.h"
#include <interface/Font.h>

namespace WebCore {

    class FontPlatformData {
    public:
        FontPlatformData(WTF::HashTableDeletedValueType)
            : m_font(hashTableDeletedFontValue())
            { }

        FontPlatformData()
            : m_font(0)
            { }

        FontPlatformData(const FontDescription&, const AtomicString& family);
        FontPlatformData(float size, bool bold, bool oblique);
        FontPlatformData(const FontPlatformData&);

        ~FontPlatformData();

        BFont* font() const { return m_font; }

        bool isFixedPitch();
        float size() const { return m_size; }
        bool bold() const { return m_bold; }
        bool oblique() const { return m_oblique; }

        unsigned hash() const;
        bool isHashTableDeletedValue() const;

        bool operator==(const FontPlatformData&) const;

#ifndef NDEBUG
        String description() const;
#endif

        BFont* m_font;
        float m_size;
        bool m_bold;
        bool m_oblique;

    private:
        static BFont* hashTableDeletedFontValue() { return reinterpret_cast<BFont*>(-1); }
    };

} // namespace WebCore

#endif

