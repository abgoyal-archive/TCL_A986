

#include "config.h"

#if ENABLE(SVG)
#include "SVGPaintServerSolid.h"

#include "GraphicsContext.h"
#include "RenderPath.h"
#include "SVGRenderTreeAsText.h"

namespace WebCore {

SVGPaintServerSolid::SVGPaintServerSolid()
{
}

SVGPaintServerSolid::~SVGPaintServerSolid()
{
}

Color SVGPaintServerSolid::color() const
{
    return m_color;
}

void SVGPaintServerSolid::setColor(const Color& color)
{
    m_color = color;
}

TextStream& SVGPaintServerSolid::externalRepresentation(TextStream& ts) const
{
    ts << "[type=SOLID]"
        << " [color="<< color() << "]";
    return ts;
}

bool SVGPaintServerSolid::setup(GraphicsContext*& context, const RenderObject* object, const RenderStyle* style, SVGPaintTargetType type, bool isPaintingText) const
{
    const SVGRenderStyle* svgStyle = style ? style->svgStyle() : 0;
    ColorSpace colorSpace = style ? style->colorSpace() : DeviceColorSpace;

    if ((type & ApplyToFillTargetType) && (!style || svgStyle->hasFill())) {
        context->setAlpha(style ? svgStyle->fillOpacity() : 1);
        context->setFillColor(color().rgb(), colorSpace);
        context->setFillRule(style ? svgStyle->fillRule() : RULE_NONZERO);

        if (isPaintingText)
            context->setTextDrawingMode(cTextFill);
    }

    if ((type & ApplyToStrokeTargetType) && (!style || svgStyle->hasStroke())) {
        context->setAlpha(style ? svgStyle->strokeOpacity() : 1);
        context->setStrokeColor(color().rgb(), colorSpace);

        if (style)
            applyStrokeStyleToContext(context, style, object);

        if (isPaintingText)
            context->setTextDrawingMode(cTextStroke);
    }

    return true;
}

} // namespace WebCore

#endif
