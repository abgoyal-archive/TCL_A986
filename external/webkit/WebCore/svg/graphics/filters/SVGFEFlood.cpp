

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEFlood.h"

#include "Filter.h"
#include "GraphicsContext.h"
#include "SVGRenderTreeAsText.h"

namespace WebCore {

FEFlood::FEFlood(const Color& floodColor, const float& floodOpacity)
    : FilterEffect()
    , m_floodColor(floodColor)
    , m_floodOpacity(floodOpacity)
{
}

PassRefPtr<FEFlood> FEFlood::create(const Color& floodColor, const float& floodOpacity)
{
    return adoptRef(new FEFlood(floodColor, floodOpacity));
}

Color FEFlood::floodColor() const
{
    return m_floodColor;
}

void FEFlood::setFloodColor(const Color& color)
{
    m_floodColor = color;
}

float FEFlood::floodOpacity() const
{
    return m_floodOpacity;
}

void FEFlood::setFloodOpacity(float floodOpacity)
{
    m_floodOpacity = floodOpacity;
}

void FEFlood::apply(Filter*)
{
    GraphicsContext* filterContext = getEffectContext();
    if (!filterContext)
        return;

    Color color = colorWithOverrideAlpha(floodColor().rgb(), floodOpacity());
    filterContext->fillRect(FloatRect(FloatPoint(), scaledSubRegion().size()), color, DeviceColorSpace);
}

void FEFlood::dump()
{
}

TextStream& FEFlood::externalRepresentation(TextStream& ts) const
{
    ts << "[type=FLOOD] ";
    FilterEffect::externalRepresentation(ts);
    ts << " [color=" << floodColor() << "]"
        << " [opacity=" << floodOpacity() << "]";
    return ts;
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
