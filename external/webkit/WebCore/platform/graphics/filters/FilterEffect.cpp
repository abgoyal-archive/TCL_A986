

#include "config.h"

#if ENABLE(FILTERS)
#include "FilterEffect.h"

namespace WebCore {

FilterEffect::FilterEffect()
    : m_hasX(false)
    , m_hasY(false)
    , m_hasWidth(false)
    , m_hasHeight(false)
    , m_alphaImage(false)
{
}

FilterEffect::~FilterEffect()
{
}

FloatRect FilterEffect::calculateUnionOfChildEffectSubregions(Filter* filter, FilterEffect* in)
{
    return in->calculateEffectRect(filter);
}

FloatRect FilterEffect::calculateUnionOfChildEffectSubregions(Filter* filter, FilterEffect* in, FilterEffect* in2)
{
    FloatRect uniteEffectRect = in->calculateEffectRect(filter);
    uniteEffectRect.unite(in2->calculateEffectRect(filter));
    return uniteEffectRect;
}

FloatRect FilterEffect::calculateEffectRect(Filter* filter)
{
    setUnionOfChildEffectSubregions(uniteChildEffectSubregions(filter));
    filter->calculateEffectSubRegion(this);
    return subRegion();
}

IntRect FilterEffect::calculateDrawingIntRect(const FloatRect& effectRect)
{
    IntPoint location = roundedIntPoint(FloatPoint(scaledSubRegion().x() - effectRect.x(),
                                                   scaledSubRegion().y() - effectRect.y()));
    return IntRect(location, resultImage()->size());
}

FloatRect FilterEffect::calculateDrawingRect(const FloatRect& srcRect)
{
    FloatPoint startPoint = FloatPoint(srcRect.x() - scaledSubRegion().x(), srcRect.y() - scaledSubRegion().y());
    FloatRect drawingRect = FloatRect(startPoint, srcRect.size());
    return drawingRect;
}

GraphicsContext* FilterEffect::getEffectContext()
{
    IntRect bufferRect = enclosingIntRect(scaledSubRegion());
    m_effectBuffer = ImageBuffer::create(bufferRect.size(), LinearRGB);
    return m_effectBuffer->context();
}

TextStream& FilterEffect::externalRepresentation(TextStream& ts) const
{
    return ts;
}

} // namespace WebCore

#endif // ENABLE(FILTERS)
