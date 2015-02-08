

#include "config.h"
#include "ImageBuffer.h"

#if !PLATFORM(CG)

#include <math.h>

namespace WebCore {

void ImageBuffer::transformColorSpace(ImageColorSpace srcColorSpace, ImageColorSpace dstColorSpace)
{
    if (srcColorSpace == dstColorSpace)
        return;

    // only sRGB <-> linearRGB are supported at the moment
    if ((srcColorSpace != LinearRGB && srcColorSpace != DeviceRGB) || 
        (dstColorSpace != LinearRGB && dstColorSpace != DeviceRGB))
        return;

    Vector<int> lookUpTable;
    if (dstColorSpace == LinearRGB) {
        if (m_linearRgbLUT.isEmpty()) {
            for (unsigned i = 0; i < 256; i++) {
                float color = i  / 255.0f;
                color = (color <= 0.04045f ? color / 12.92f : pow((color + 0.055f) / 1.055f, 2.4f));
                color = std::max(0.0f, color);
                color = std::min(1.0f, color);
                m_linearRgbLUT.append(static_cast<int>(color * 255));
            }
        }
        platformTransformColorSpace(m_linearRgbLUT);
    } else if (dstColorSpace == DeviceRGB) {
        if (m_deviceRgbLUT.isEmpty()) {
            for (unsigned i = 0; i < 256; i++) {
                float color = i  / 255.0f;
                color = pow(1.055f * color, 1.0f / 2.4f) - 0.055f;
                color = std::max(0.0f, color);
                color = std::min(1.0f, color);
                m_deviceRgbLUT.append(static_cast<int>(color * 255));
            }
        }
        platformTransformColorSpace(m_deviceRgbLUT);
    }
}

}

#endif // PLATFORM(CG)
