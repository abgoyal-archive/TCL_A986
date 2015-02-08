

#include "config.h"

#if ENABLE(SVG)
#include "SVGViewElement.h"

#include "Attr.h"
#include "MappedAttribute.h"
#include "PlatformString.h"
#include "SVGFitToViewBox.h"
#include "SVGNames.h"
#include "SVGStringList.h"
#include "SVGZoomAndPan.h"

namespace WebCore {

SVGViewElement::SVGViewElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledElement(tagName, doc)
    , SVGExternalResourcesRequired()
    , SVGFitToViewBox()
    , SVGZoomAndPan()
{
}

SVGViewElement::~SVGViewElement()
{
}

SVGStringList* SVGViewElement::viewTarget() const
{
    if (!m_viewTarget)
        m_viewTarget = SVGStringList::create(SVGNames::viewTargetAttr);

    return m_viewTarget.get();
}

void SVGViewElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::viewTargetAttr)
        viewTarget()->reset(attr->value());
    else {
        if (SVGExternalResourcesRequired::parseMappedAttribute(attr)
           || SVGFitToViewBox::parseMappedAttribute(document(), attr)
           || SVGZoomAndPan::parseMappedAttribute(attr))
            return;

        SVGStyledElement::parseMappedAttribute(attr);
    }
}

void SVGViewElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeExternalResourcesRequired();
        synchronizeViewBox();
        synchronizePreserveAspectRatio();
        return;
    }

    if (SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
    else if (SVGFitToViewBox::isKnownAttribute(attrName)) {
        synchronizeViewBox();
        synchronizePreserveAspectRatio();
    }
}

}

#endif // ENABLE(SVG)
