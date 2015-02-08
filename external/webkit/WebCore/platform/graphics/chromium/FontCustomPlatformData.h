

#ifndef FontCustomPlatformData_h
#define FontCustomPlatformData_h

#include "FontRenderingMode.h"
#include <wtf/Noncopyable.h>

#if OS(WINDOWS)
#include "PlatformString.h"
#include <windows.h>
#elif OS(LINUX)
#include "SkTypeface.h"
#endif

namespace WebCore {

class FontPlatformData;
class SharedBuffer;

struct FontCustomPlatformData : Noncopyable {
#if OS(WINDOWS)
    FontCustomPlatformData(HANDLE fontReference, const String& name)
        : m_fontReference(fontReference)
        , m_name(name)
    {}
#elif OS(LINUX)
    explicit FontCustomPlatformData(SkTypeface* typeface)
        : m_fontReference(typeface)
    {}
#endif

    ~FontCustomPlatformData();

    FontPlatformData fontPlatformData(int size, bool bold, bool italic,
                                      FontRenderingMode = NormalRenderingMode);

#if OS(WINDOWS)
    HANDLE m_fontReference;
    String m_name;
#elif OS(LINUX)
    SkTypeface* m_fontReference;
#endif
};

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer*);
}

#endif // FontCustomPlatformData_h
