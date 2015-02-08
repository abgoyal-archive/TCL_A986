

#include "config.h"
#if ENABLE(SVG)
#include "SVGSymbolElement.h"

#include "PlatformString.h"
#include "SVGFitToViewBox.h"

namespace WebCore {

SVGSymbolElement::SVGSymbolElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledElement(tagName, doc)
    , SVGLangSpace()
    , SVGExternalResourcesRequired()
    , SVGFitToViewBox()
{
}

SVGSymbolElement::~SVGSymbolElement()
{
}

void SVGSymbolElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (SVGLangSpace::parseMappedAttribute(attr))
        return;
    if (SVGExternalResourcesRequired::parseMappedAttribute(attr))
        return;
    if (SVGFitToViewBox::parseMappedAttribute(document(), attr))
        return;

    SVGStyledElement::parseMappedAttribute(attr);
}

void SVGSymbolElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizePreserveAspectRatio();
        synchronizeViewBox();
        synchronizeExternalResourcesRequired();
        synchronizeViewBox();
        synchronizePreserveAspectRatio();
        return;
    }

    if (attrName == SVGNames::preserveAspectRatioAttr)
        synchronizePreserveAspectRatio();
    else if (attrName == SVGNames::viewBoxAttr)
        synchronizeViewBox();
    else if (SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
    else if (SVGFitToViewBox::isKnownAttribute(attrName)) {
        synchronizeViewBox();
        synchronizePreserveAspectRatio();
    } 
}

}

#endif // ENABLE(SVG)
