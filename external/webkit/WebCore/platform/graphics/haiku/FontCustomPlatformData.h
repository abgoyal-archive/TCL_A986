

#ifndef FontCustomPlatformData_h
#define FontCustomPlatformData_h

#include "FontRenderingMode.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

    class FontPlatformData;
    class SharedBuffer;

    struct FontCustomPlatformData : Noncopyable {
    public:
        FontCustomPlatformData() { }
        ~FontCustomPlatformData();

        FontPlatformData fontPlatformData(int size, bool bold, bool italic, FontRenderingMode = NormalRenderingMode);
    };

    FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer*);

}

#endif
