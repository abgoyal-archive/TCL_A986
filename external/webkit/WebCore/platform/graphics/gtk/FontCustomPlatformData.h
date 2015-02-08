

#ifndef FontCustomPlatformData_h
#define FontCustomPlatformData_h

#include "FontRenderingMode.h"
#include <wtf/Noncopyable.h>

typedef struct _cairo_font_face cairo_font_face_t;

namespace WebCore {

class FontPlatformData;
class SharedBuffer;

struct FontCustomPlatformData : Noncopyable {
    FontCustomPlatformData(cairo_font_face_t* fontFace)
    : m_fontFace(fontFace)
    {}

    ~FontCustomPlatformData();

    FontPlatformData fontPlatformData(int size, bool bold, bool italic, FontRenderingMode = NormalRenderingMode);

    cairo_font_face_t* m_fontFace;
};

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer* buffer);

}

#endif
