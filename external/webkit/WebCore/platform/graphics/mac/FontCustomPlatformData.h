

#ifndef FontCustomPlatformData_h
#define FontCustomPlatformData_h

#include "FontRenderingMode.h"
#include <CoreFoundation/CFBase.h>
#include <wtf/Noncopyable.h>

typedef struct CGFont* CGFontRef;
typedef UInt32 ATSFontContainerRef;
typedef UInt32 ATSFontRef;

namespace WebCore {

class FontPlatformData;
class SharedBuffer;

struct FontCustomPlatformData : Noncopyable {
    FontCustomPlatformData(ATSFontContainerRef container, ATSFontRef atsFont, CGFontRef cgFont)
    : m_atsContainer(container), m_atsFont(atsFont), m_cgFont(cgFont)
    {}
    ~FontCustomPlatformData();

    FontPlatformData fontPlatformData(int size, bool bold, bool italic, FontRenderingMode = NormalRenderingMode);

    ATSFontContainerRef m_atsContainer;
    ATSFontRef m_atsFont;
    CGFontRef m_cgFont;
};

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer* buffer);

}

#endif
