

#include "config.h"

#if ENABLE(SVG)
#include "SVGTextElement.h"

#include "AffineTransform.h"
#include "FloatRect.h"
#include "MappedAttribute.h"
#include "RenderSVGText.h"
#include "SVGLengthList.h"
#include "SVGRenderStyle.h"
#include "SVGTSpanElement.h"
#include "SVGTransformList.h"

namespace WebCore {

SVGTextElement::SVGTextElement(const QualifiedName& tagName, Document* doc)
    : SVGTextPositioningElement(tagName, doc)
    , SVGTransformable()
    , m_transform(SVGTransformList::create(SVGNames::transformAttr))
{
}

SVGTextElement::~SVGTextElement()
{
}

void SVGTextElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (SVGTransformable::isKnownAttribute(attr->name())) {
        SVGTransformList* localTransforms = transformBaseValue();
        if (!SVGTransformable::parseTransformAttribute(localTransforms, attr->value())) {
            ExceptionCode ec = 0;
            localTransforms->clear(ec);
        }
    } else
        SVGTextPositioningElement::parseMappedAttribute(attr);
}

SVGElement* SVGTextElement::nearestViewportElement() const
{
    return SVGTransformable::nearestViewportElement(this);
}

SVGElement* SVGTextElement::farthestViewportElement() const
{
    return SVGTransformable::farthestViewportElement(this);
}

FloatRect SVGTextElement::getBBox() const
{
    return SVGTransformable::getBBox(this);
}

AffineTransform SVGTextElement::getScreenCTM() const
{
    return SVGTransformable::getScreenCTM(this);
}

AffineTransform SVGTextElement::getCTM() const
{
    return SVGTransformable::getCTM(this);
}

AffineTransform SVGTextElement::animatedLocalTransform() const
{
    return m_supplementalTransform ? transform()->concatenate().matrix() * *m_supplementalTransform : transform()->concatenate().matrix();
}

AffineTransform* SVGTextElement::supplementalTransform()
{
    if (!m_supplementalTransform)
        m_supplementalTransform.set(new AffineTransform());
    return m_supplementalTransform.get();
}

RenderObject* SVGTextElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGText(this);
}

bool SVGTextElement::childShouldCreateRenderer(Node* child) const
{
    if (child->isTextNode()
#if ENABLE(SVG_FONTS)
        || child->hasTagName(SVGNames::altGlyphTag)
#endif
        || child->hasTagName(SVGNames::tspanTag) || child->hasTagName(SVGNames::trefTag) || child->hasTagName(SVGNames::aTag) || child->hasTagName(SVGNames::textPathTag))
        return true;
    return false;
}

void SVGTextElement::svgAttributeChanged(const QualifiedName& attrName)
{
    SVGTextPositioningElement::svgAttributeChanged(attrName);

    if (!renderer())
        return;

    if (SVGTransformable::isKnownAttribute(attrName))
        renderer()->setNeedsLayout(true);
}

void SVGTextElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGTextPositioningElement::synchronizeProperty(attrName);

    if (attrName == anyQName() || SVGTransformable::isKnownAttribute(attrName))
        synchronizeTransform();
}

void SVGTextElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    SVGTextPositioningElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);

    if (!renderer())
        return;

    renderer()->setNeedsLayout(true);
}

}

#endif // ENABLE(SVG)
