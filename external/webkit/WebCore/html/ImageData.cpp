

#include "config.h"
#include "ImageData.h"

namespace WebCore {

PassRefPtr<ImageData> ImageData::create(unsigned width, unsigned height)
{
    return adoptRef(new ImageData(width, height));
}

ImageData::ImageData(unsigned width, unsigned height)
    : m_width(width)
    , m_height(height)
    , m_data(CanvasPixelArray::create(width * height * 4))
{
}

}

