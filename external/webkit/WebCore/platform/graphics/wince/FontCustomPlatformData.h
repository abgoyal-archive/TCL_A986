

#ifndef FontCustomPlatformData_h
#define FontCustomPlatformData_h

#include "FontRenderingMode.h"
#include "PlatformString.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

    class FontPlatformData;
    class CachedFont;

    class CustomFontCache {
    public:
        virtual bool registerFont(const String& fontName, const SharedBuffer*) = 0;
        virtual void unregisterFont(const String& fontName) = 0;
    };

    struct FontCustomPlatformData : Noncopyable {
        FontCustomPlatformData(const String& name)
            : m_name(name)
        {
        }

        ~FontCustomPlatformData();

        FontPlatformData fontPlatformData(int size, bool bold, bool italic, FontRenderingMode = NormalRenderingMode);
        String m_name;
    };

    FontCustomPlatformData* createFontCustomPlatformData(const SharedBuffer*);
    void setCustomFontCache(CustomFontCache*);
}

#endif
