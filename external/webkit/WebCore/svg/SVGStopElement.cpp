

#include "config.h"

#if ENABLE(SVG)
#include "SVGStopElement.h"

#include "Document.h"
#include "MappedAttribute.h"
#include "RenderSVGGradientStop.h"
#include "SVGGradientElement.h"
#include "SVGNames.h"

namespace WebCore {

SVGStopElement::SVGStopElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledElement(tagName, doc)
    , m_offset(0.0f)
{
}

SVGStopElement::~SVGStopElement()
{
}

void SVGStopElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::offsetAttr) {
        const String& value = attr->value();
        if (value.endsWith("%"))
            setOffsetBaseValue(value.left(value.length() - 1).toFloat() / 100.0f);
        else
            setOffsetBaseValue(value.toFloat());

        setNeedsStyleRecalc();
    } else
        SVGStyledElement::parseMappedAttribute(attr);
}

void SVGStopElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledElement::synchronizeProperty(attrName);

    if (attrName == anyQName() || attrName == SVGNames::offsetAttr)
        synchronizeOffset();
}

RenderObject* SVGStopElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGGradientStop(this);
}

}

#endif // ENABLE(SVG)
