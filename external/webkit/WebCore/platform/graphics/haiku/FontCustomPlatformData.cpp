

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
    return FontPlatformData(size, bold, italic);
}

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer* buffer)
{
    // FIXME: We need support in Haiku to read fonts from memory to implement this.
    return 0;
}

}
