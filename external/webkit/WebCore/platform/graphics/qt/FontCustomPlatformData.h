
#ifndef FontCustomPlatformData_h_
#define FontCustomPlatformData_h_

#include "FontRenderingMode.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

class SharedBuffer;
class FontPlatformData;

struct FontCustomPlatformData : Noncopyable {
    ~FontCustomPlatformData();

    // for use with QFontDatabase::addApplicationFont/removeApplicationFont
    int m_handle;

    FontPlatformData fontPlatformData(int size, bool bold, bool italic, FontRenderingMode = NormalRenderingMode);
};

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer* buffer);

} // namespace WebCore

#endif // FontCustomPlatformData_h_
