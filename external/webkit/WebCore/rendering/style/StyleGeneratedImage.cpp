

#include "config.h"
#include "StyleGeneratedImage.h"

#include "CSSImageGeneratorValue.h"
#include "RenderObject.h"

namespace WebCore {

PassRefPtr<CSSValue> StyleGeneratedImage::cssValue()
{
    return m_generator;
}

IntSize StyleGeneratedImage::imageSize(const RenderObject* renderer, float multiplier) const
{
    if (m_fixedSize) {
        IntSize fixedSize = m_generator->fixedSize(renderer);
        if (multiplier == 1.0f)
            return fixedSize;

        int width = fixedSize.width() * multiplier;
        int height = fixedSize.height() * multiplier;

        // Don't let images that have a width/height >= 1 shrink below 1 when zoomed.
        if (fixedSize.width() > 0)
            width = max(1, width);

        if (fixedSize.height() > 0)
            height = max(1, height);

        return IntSize(width, height);
    }
    
    return m_containerSize;
}

void StyleGeneratedImage::setImageContainerSize(const IntSize& size)
{
    m_containerSize = size;
}

void StyleGeneratedImage::addClient(RenderObject* renderer)
{
    m_generator->addClient(renderer, IntSize());
}

void StyleGeneratedImage::removeClient(RenderObject* renderer)
{
    m_generator->removeClient(renderer);
}

Image* StyleGeneratedImage::image(RenderObject* renderer, const IntSize& size) const
{
    return m_generator->image(renderer, size);
}

}
