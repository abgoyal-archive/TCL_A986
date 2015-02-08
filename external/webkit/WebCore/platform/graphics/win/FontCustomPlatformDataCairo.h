

#ifndef FontCustomPlatformDataCairo_h
#define FontCustomPlatformDataCairo_h

#include <wtf/Noncopyable.h>

#include <cairo.h>

namespace WebCore {

class FontPlatformData;
class SharedBuffer;

struct FontCustomPlatformDataCairo : Noncopyable {
    FontCustomPlatformDataCairo(cairo_font_face_t* fontFace)
        : m_fontFace(fontFace)
    {
    }
    ~FontCustomPlatformDataCairo();

    FontPlatformData fontPlatformData(int size, bool bold, bool italic);

    cairo_font_face_t* m_fontFace;
};

FontCustomPlatformDataCairo* createFontCustomPlatformData(SharedBuffer*);

}

#endif
