

#include "config.h"

#if ENABLE(SVG)
#include "SVGPaintServerRadialGradient.h"
#include "SVGRenderTreeAsText.h"

namespace WebCore {

SVGPaintServerRadialGradient::SVGPaintServerRadialGradient(const SVGGradientElement* owner)
    : SVGPaintServerGradient(owner)
    , m_radius(0.0f)
{
}

SVGPaintServerRadialGradient::~SVGPaintServerRadialGradient()
{
}


FloatPoint SVGPaintServerRadialGradient::gradientCenter() const
{
    return m_center;
}

void SVGPaintServerRadialGradient::setGradientCenter(const FloatPoint& center)
{
    m_center = center;
}

FloatPoint SVGPaintServerRadialGradient::gradientFocal() const
{
    return m_focal;
}

void SVGPaintServerRadialGradient::setGradientFocal(const FloatPoint& focal)
{
    m_focal = focal;
}

float SVGPaintServerRadialGradient::gradientRadius() const
{
    return m_radius;
}

void SVGPaintServerRadialGradient::setGradientRadius(float radius)
{
    m_radius = radius;
}

TextStream& SVGPaintServerRadialGradient::externalRepresentation(TextStream& ts) const
{
    ts << "[type=RADIAL-GRADIENT] ";
    SVGPaintServerGradient::externalRepresentation(ts);
    ts << " [center=" << gradientCenter() << "]"
        << " [focal=" << gradientFocal() << "]"
        << " [radius=" << gradientRadius() << "]";
    return ts;
}

} // namespace WebCore

#endif
