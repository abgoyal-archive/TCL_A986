

#include "config.h"

#if ENABLE(SVG)
#include "SVGTextPathElement.h"

#include "AffineTransform.h"
#include "FloatRect.h"
#include "MappedAttribute.h"
#include "RenderSVGTextPath.h"
#include "SVGLengthList.h"
#include "SVGPathElement.h"
#include "SVGRenderStyle.h"
#include "SVGTransformList.h"

namespace WebCore {

SVGTextPathElement::SVGTextPathElement(const QualifiedName& tagName, Document* doc)
    : SVGTextContentElement(tagName, doc)
    , SVGURIReference()
    , m_startOffset(LengthModeOther)
    , m_method(SVG_TEXTPATH_METHODTYPE_ALIGN)
    , m_spacing(SVG_TEXTPATH_SPACINGTYPE_EXACT)
{
}

SVGTextPathElement::~SVGTextPathElement()
{
}

void SVGTextPathElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();

    if (attr->name() == SVGNames::startOffsetAttr)
        setStartOffsetBaseValue(SVGLength(LengthModeOther, value));
    else if (attr->name() == SVGNames::methodAttr) {
        if (value == "align")
            setSpacingBaseValue(SVG_TEXTPATH_METHODTYPE_ALIGN);
        else if (value == "stretch")
            setSpacingBaseValue(SVG_TEXTPATH_METHODTYPE_STRETCH);
    } else if (attr->name() == SVGNames::spacingAttr) {
        if (value == "auto")
            setMethodBaseValue(SVG_TEXTPATH_SPACINGTYPE_AUTO);
        else if (value == "exact")
            setMethodBaseValue(SVG_TEXTPATH_SPACINGTYPE_EXACT);
    } else {
        if (SVGURIReference::parseMappedAttribute(attr))
            return;
        SVGTextContentElement::parseMappedAttribute(attr);
    }
}

void SVGTextPathElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGTextContentElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeStartOffset();
        synchronizeMethod();
        synchronizeSpacing();
        synchronizeHref();
        return;
    }

    if (attrName == SVGNames::startOffsetAttr)
        synchronizeStartOffset();
    else if (attrName == SVGNames::methodAttr)
        synchronizeMethod();
    else if (attrName == SVGNames::spacingAttr)
        synchronizeSpacing();
    else if (SVGURIReference::isKnownAttribute(attrName))
        synchronizeHref();
}

RenderObject* SVGTextPathElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGTextPath(this);
}

bool SVGTextPathElement::childShouldCreateRenderer(Node* child) const
{
    if (child->isTextNode()
#if ENABLE(SVG_FONTS)
        || child->hasTagName(SVGNames::altGlyphTag)
#endif
        || child->hasTagName(SVGNames::trefTag) || child->hasTagName(SVGNames::tspanTag) || child->hasTagName(SVGNames::textPathTag))
        return true;

    return false;
}

void SVGTextPathElement::insertedIntoDocument()
{
    SVGElement::insertedIntoDocument();

    String id = SVGURIReference::getTarget(href());
    Element* targetElement = ownerDocument()->getElementById(id);
    if (!targetElement) {
        document()->accessSVGExtensions()->addPendingResource(id, this);
        return;
    }
}

}

#endif // ENABLE(SVG)
