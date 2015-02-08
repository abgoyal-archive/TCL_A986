

#include "config.h"

#if ENABLE(SVG)
#include "SVGCursorElement.h"

#include "Attr.h"
#include "Document.h"
#include "MappedAttribute.h"
#include "SVGLength.h"
#include "SVGNames.h"

namespace WebCore {

SVGCursorElement::SVGCursorElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
    , SVGTests()
    , SVGExternalResourcesRequired()
    , SVGURIReference()
    , m_x(LengthModeWidth)
    , m_y(LengthModeHeight)
{
}

SVGCursorElement::~SVGCursorElement()
{
    HashSet<SVGElement*>::iterator end = m_clients.end();
    for (HashSet<SVGElement*>::iterator it = m_clients.begin(); it != end; ++it)
        (*it)->setCursorElement(0);
}

void SVGCursorElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::xAttr)
        setXBaseValue(SVGLength(LengthModeWidth, attr->value()));
    else if (attr->name() == SVGNames::yAttr)
        setYBaseValue(SVGLength(LengthModeHeight, attr->value()));
    else {
        if (SVGTests::parseMappedAttribute(attr))
            return;
        if (SVGExternalResourcesRequired::parseMappedAttribute(attr))
            return;
        if (SVGURIReference::parseMappedAttribute(attr))
            return;

        SVGElement::parseMappedAttribute(attr);
    }
}

void SVGCursorElement::addClient(SVGElement* element)
{
    m_clients.add(element);
    element->setCursorElement(this);
}

void SVGCursorElement::removeClient(SVGElement* element)
{
    m_clients.remove(element);
    element->setCursorElement(0);
}

void SVGCursorElement::svgAttributeChanged(const QualifiedName& attrName)
{
    SVGElement::svgAttributeChanged(attrName);

    if (attrName == SVGNames::xAttr || attrName == SVGNames::yAttr ||
        SVGTests::isKnownAttribute(attrName) ||
        SVGExternalResourcesRequired::isKnownAttribute(attrName) ||
        SVGURIReference::isKnownAttribute(attrName)) {
        HashSet<SVGElement*>::const_iterator it = m_clients.begin();
        HashSet<SVGElement*>::const_iterator end = m_clients.end();

        for (; it != end; ++it)
            (*it)->setNeedsStyleRecalc();
    }
}

void SVGCursorElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeX();
        synchronizeY();
        synchronizeExternalResourcesRequired();
        synchronizeHref();
        return;
    }

    if (attrName == SVGNames::xAttr)
        synchronizeX();
    else if (attrName == SVGNames::yAttr)
        synchronizeY();
    else if (SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
    else if (SVGURIReference::isKnownAttribute(attrName))
        synchronizeHref();
}

void SVGCursorElement::addSubresourceAttributeURLs(ListHashSet<KURL>& urls) const
{
    SVGElement::addSubresourceAttributeURLs(urls);

    addSubresourceURL(urls, document()->completeURL(href()));
}

}

#endif // ENABLE(SVG)
