

#include "config.h"
#include "FontCustomPlatformData.h"

#include "SharedBuffer.h"
#include "FontPlatformData.h"

namespace WebCore {

FontCustomPlatformData::~FontCustomPlatformData()
{
}

FontPlatformData FontCustomPlatformData::fontPlatformData(int size, bool bold, bool italic, FontRenderingMode)
{
    return FontPlatformData(m_fontFace, size, bold, italic);
}

static void releaseData(void* data)
{
    static_cast<SharedBuffer*>(data)->deref();
}

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer* buffer)
{
    // FIXME: we need support in pango to read fonts from memory to implement this.y
    return 0;
}

}
