

#include "config.h"
#include "FontCustomPlatformDataCairo.h"

#include "SharedBuffer.h"
#include "FontPlatformData.h"
#include <wtf/RetainPtr.h>

namespace WebCore {

FontCustomPlatformDataCairo::~FontCustomPlatformDataCairo()
{
   cairo_font_face_destroy(m_fontFace);
}

FontPlatformData FontCustomPlatformDataCairo::fontPlatformData(int size, bool bold, bool italic)
{
    return FontPlatformData(m_fontFace, size, bold, italic);
}

static void releaseData(void* data)
{
    static_cast<SharedBuffer*>(data)->deref();
}

FontCustomPlatformDataCairo* createFontCustomPlatformData(SharedBuffer* buffer)
{
    ASSERT_ARG(buffer, buffer);

    buffer->ref();
    HFONT font = reinterpret_cast<HFONT>(buffer);
    cairo_font_face_t* fontFace = cairo_win32_font_face_create_for_hfont(font);
    if (!fontFace)
       return 0;

    static cairo_user_data_key_t bufferKey;
    cairo_font_face_set_user_data(fontFace, &bufferKey, buffer, releaseData);

    return new FontCustomPlatformDataCairo(fontFace);
}

}
