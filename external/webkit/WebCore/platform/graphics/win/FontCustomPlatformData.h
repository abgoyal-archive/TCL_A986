

#ifndef FontCustomPlatformData_h
#define FontCustomPlatformData_h

#include "FontRenderingMode.h"
#include "PlatformString.h"
#include <wtf/Noncopyable.h>

typedef struct CGFont* CGFontRef;

namespace WebCore {

class FontPlatformData;
class SharedBuffer;

struct FontCustomPlatformData : Noncopyable {
    FontCustomPlatformData(CGFontRef cgFont, HANDLE fontReference, const String& name)
        : m_cgFont(cgFont)
        , m_fontReference(fontReference)
        , m_name(name)
    {
    }

    ~FontCustomPlatformData();

    FontPlatformData fontPlatformData(int size, bool bold, bool italic, FontRenderingMode = NormalRenderingMode);

    CGFontRef m_cgFont;
    HANDLE m_fontReference;
    String m_name;
};

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer*);

}

#endif
