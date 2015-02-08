

#include "config.h"
#include "StyleCachedImage.h"

#include "CachedImage.h"
#include "RenderObject.h"

namespace WebCore {

PassRefPtr<CSSValue> StyleCachedImage::cssValue()
{
    return CSSPrimitiveValue::create(m_image->url(), CSSPrimitiveValue::CSS_URI);
}

bool StyleCachedImage::canRender(float multiplier) const
{
    return m_image->canRender(multiplier);
}

bool StyleCachedImage::isLoaded() const
{
    return m_image->isLoaded();
}

bool StyleCachedImage::errorOccurred() const
{
    return m_image->errorOccurred();
}

IntSize StyleCachedImage::imageSize(const RenderObject* /*renderer*/, float multiplier) const
{
    return m_image->imageSize(multiplier);
}

bool StyleCachedImage::imageHasRelativeWidth() const
{
    return m_image->imageHasRelativeWidth();
}

bool StyleCachedImage::imageHasRelativeHeight() const
{
    return m_image->imageHasRelativeHeight();
}

bool StyleCachedImage::usesImageContainerSize() const
{
    return m_image->usesImageContainerSize();
}

void StyleCachedImage::setImageContainerSize(const IntSize& size)
{
    return m_image->setImageContainerSize(size);
}

void StyleCachedImage::addClient(RenderObject* renderer)
{
    return m_image->addClient(renderer);
}

void StyleCachedImage::removeClient(RenderObject* renderer)
{
    return m_image->removeClient(renderer);
}

Image* StyleCachedImage::image(RenderObject*, const IntSize&) const
{
    return m_image->image();
}

}
