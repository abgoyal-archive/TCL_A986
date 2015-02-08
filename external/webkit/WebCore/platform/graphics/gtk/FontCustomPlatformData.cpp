

#include "config.h"
#include "FontCustomPlatformData.h"

#include "SharedBuffer.h"
#include "FontPlatformData.h"

namespace WebCore {

FontCustomPlatformData::~FontCustomPlatformData()
{
    cairo_font_face_destroy(m_fontFace);
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
    ASSERT_ARG(buffer, buffer);

    int error;

    static FT_Library library = 0;
    if (!library) {
        error = FT_Init_FreeType(&library);
        if (error) {
            library = 0;
            return 0;
        }
    }

    FT_Face face;
    error = FT_New_Memory_Face(library, reinterpret_cast<const FT_Byte*>(buffer->data()), buffer->size(), 0, &face);
    if (error)
        return 0;

    buffer->ref();
    cairo_font_face_t* fontFace = cairo_ft_font_face_create_for_ft_face(face, 0);

    static cairo_user_data_key_t bufferKey;
    cairo_font_face_set_user_data(fontFace, &bufferKey, buffer, releaseData);

    return new FontCustomPlatformData(fontFace);
}

}
