

#include "config.h"
#include "ImageDecoder.h"

#include <cairo.h>

namespace WebCore {

NativeImagePtr RGBA32Buffer::asNewNativeImage() const
{
    return cairo_image_surface_create_for_data(
        reinterpret_cast<unsigned char*>(const_cast<PixelData*>(
            m_bytes.data())), CAIRO_FORMAT_ARGB32, width(), height(),
        width() * sizeof(PixelData));
}

} // namespace WebCore
