

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEImageElement.h"

#include "Attr.h"
#include "CachedImage.h"
#include "DocLoader.h"
#include "Document.h"
#include "MappedAttribute.h"
#include "SVGLength.h"
#include "SVGNames.h"
#include "SVGPreserveAspectRatio.h"
#include "SVGRenderSupport.h"
#include "SVGResourceFilter.h"

namespace WebCore {

SVGFEImageElement::SVGFEImageElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
    , SVGURIReference()
    , SVGLangSpace()
    , SVGExternalResourcesRequired()
{
}

SVGFEImageElement::~SVGFEImageElement()
{
    if (m_cachedImage)
        m_cachedImage->removeClient(this);
}

void SVGFEImageElement::requestImageResource()
{
    if (m_cachedImage) {
        m_cachedImage->removeClient(this);
        m_cachedImage = 0;
    }

    Element* hrefElement = document()->getElementById(SVGURIReference::getTarget(href()));
    if (hrefElement && hrefElement->isSVGElement() && hrefElement->renderer())
        return;

    m_cachedImage = ownerDocument()->docLoader()->requestImage(href());

    if (m_cachedImage)
        m_cachedImage->addClient(this);
}

void SVGFEImageElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::preserveAspectRatioAttr)
        SVGPreserveAspectRatio::parsePreserveAspectRatio(this, value);
    else {
        if (SVGURIReference::parseMappedAttribute(attr)) {
            requestImageResource();
            return;
        }
        if (SVGLangSpace::parseMappedAttribute(attr))
            return;
        if (SVGExternalResourcesRequired::parseMappedAttribute(attr))
            return;

        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
    }
}

void SVGFEImageElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizePreserveAspectRatio();
        synchronizeHref();
        synchronizeExternalResourcesRequired();
        return;
    }

    if (attrName == SVGNames::preserveAspectRatioAttr)
        synchronizePreserveAspectRatio();
    else if (SVGURIReference::isKnownAttribute(attrName))
        synchronizeHref();
    else if (SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
}

void SVGFEImageElement::notifyFinished(CachedResource*)
{
    SVGStyledElement::invalidateResourcesInAncestorChain();
}

bool SVGFEImageElement::build(SVGResourceFilter* filterResource)
{
    if (!m_cachedImage && !m_targetImage) {
        Element* hrefElement = document()->getElementById(SVGURIReference::getTarget(href()));
        if (!hrefElement || !hrefElement->isSVGElement())
            return false;

        RenderObject* renderer = hrefElement->renderer();
        if (!renderer)
            return false;

        IntRect targetRect = enclosingIntRect(renderer->objectBoundingBox());
        m_targetImage = ImageBuffer::create(targetRect.size(), LinearRGB);

        renderSubtreeToImage(m_targetImage.get(), renderer);
    }

    RefPtr<FilterEffect> effect = FEImage::create(m_targetImage ? m_targetImage->image() : m_cachedImage->image(), preserveAspectRatio());
    filterResource->addFilterEffect(this, effect.release());

    return true;
}

void SVGFEImageElement::addSubresourceAttributeURLs(ListHashSet<KURL>& urls) const
{
    SVGFilterPrimitiveStandardAttributes::addSubresourceAttributeURLs(urls);

    addSubresourceURL(urls, document()->completeURL(href()));
}

}

#endif // ENABLE(SVG)
