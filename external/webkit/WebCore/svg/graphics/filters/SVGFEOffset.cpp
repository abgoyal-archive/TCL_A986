

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEOffset.h"

#include "Filter.h"
#include "GraphicsContext.h"
#include "SVGRenderTreeAsText.h"

namespace WebCore {

FEOffset::FEOffset(FilterEffect* in, const float& dx, const float& dy)
    : FilterEffect()
    , m_in(in)
    , m_dx(dx)
    , m_dy(dy)
{
}

PassRefPtr<FEOffset> FEOffset::create(FilterEffect* in, const float& dx, const float& dy)
{
    return adoptRef(new FEOffset(in, dx, dy));
}

float FEOffset::dx() const
{
    return m_dx;
}

void FEOffset::setDx(float dx)
{
    m_dx = dx;
}

float FEOffset::dy() const
{
    return m_dy;
}

void FEOffset::setDy(float dy)
{
    m_dy = dy;
}

void FEOffset::apply(Filter* filter)
{
    m_in->apply(filter);
    if (!m_in->resultImage())
        return;

    GraphicsContext* filterContext = getEffectContext();
    if (!filterContext)
        return;

    setIsAlphaImage(m_in->isAlphaImage());

    FloatRect sourceImageRect = filter->sourceImageRect();
    sourceImageRect.scale(filter->filterResolution().width(), filter->filterResolution().height());

    if (filter->effectBoundingBoxMode()) {
        m_dx *= sourceImageRect.width();
        m_dy *= sourceImageRect.height();
    }
    m_dx *= filter->filterResolution().width();
    m_dy *= filter->filterResolution().height();

    FloatRect dstRect = FloatRect(m_dx + m_in->scaledSubRegion().x() - scaledSubRegion().x(),
                                  m_dy + m_in->scaledSubRegion().y() - scaledSubRegion().y(),
                                  m_in->scaledSubRegion().width(),
                                  m_in->scaledSubRegion().height());

    filterContext->drawImage(m_in->resultImage()->image(), DeviceColorSpace, dstRect);
}

void FEOffset::dump()
{
}

TextStream& FEOffset::externalRepresentation(TextStream& ts) const
{
    ts << "[type=OFFSET] "; 
    FilterEffect::externalRepresentation(ts);
    ts << " [dx=" << dx() << " dy=" << dy() << "]";
    return ts;
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
