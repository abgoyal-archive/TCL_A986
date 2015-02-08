

#include "config.h"

#if ENABLE(SVG)
#include "SVGPaintServerLinearGradient.h"
#include "SVGRenderTreeAsText.h"

namespace WebCore {

SVGPaintServerLinearGradient::SVGPaintServerLinearGradient(const SVGGradientElement* owner)
    : SVGPaintServerGradient(owner)
{ 
}

SVGPaintServerLinearGradient::~SVGPaintServerLinearGradient()
{
}

FloatPoint SVGPaintServerLinearGradient::gradientStart() const
{
    return m_start;
}

void SVGPaintServerLinearGradient::setGradientStart(const FloatPoint& start)
{
    m_start = start;
}

FloatPoint SVGPaintServerLinearGradient::gradientEnd() const
{
    return m_end;
}

void SVGPaintServerLinearGradient::setGradientEnd(const FloatPoint& end)
{
    m_end = end;
}

TextStream& SVGPaintServerLinearGradient::externalRepresentation(TextStream& ts) const
{
    ts << "[type=LINEAR-GRADIENT] ";
    SVGPaintServerGradient::externalRepresentation(ts);
    ts  << " [start=" << gradientStart() << "]"
        << " [end=" << gradientEnd() << "]";
    return ts;
}

} // namespace WebCore

#endif
