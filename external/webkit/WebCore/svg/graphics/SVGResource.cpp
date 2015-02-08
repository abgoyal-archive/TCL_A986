

#include "config.h"

#if ENABLE(SVG)
#include "SVGResource.h"

#include "RenderPath.h"
#include "SVGElement.h"
#include "SVGStyledElement.h"
#include <wtf/HashSet.h>
#include <wtf/StdLibExtras.h>

namespace WebCore {

typedef HashSet<SVGResource*> ResourceSet;

static ResourceSet& resourceSet()
{
    DEFINE_STATIC_LOCAL(ResourceSet, set, ());
    return set;
}

SVGResource::SVGResource()
{
    ASSERT(!resourceSet().contains(this));
    resourceSet().add(this);
}


SVGResource::~SVGResource()
{
    ASSERT(resourceSet().contains(this));
    resourceSet().remove(this);
}

void SVGResource::invalidate()
{
    HashSet<SVGStyledElement*>::const_iterator it = m_clients.begin();
    const HashSet<SVGStyledElement*>::const_iterator end = m_clients.end();

    for (; it != end; ++it) {
        SVGStyledElement* cur = *it;

        if (cur->renderer())
            cur->renderer()->setNeedsLayout(true);

        cur->invalidateResourcesInAncestorChain();
    }
}

void SVGResource::invalidateClients(HashSet<SVGStyledElement*> clients)
{
    HashSet<SVGStyledElement*>::const_iterator it = clients.begin();
    const HashSet<SVGStyledElement*>::const_iterator end = clients.end();

    for (; it != end; ++it) {
        SVGStyledElement* cur = *it;

        if (cur->renderer())
            cur->renderer()->setNeedsLayout(true);

        cur->invalidateResourcesInAncestorChain();
    }
}

void SVGResource::removeClient(SVGStyledElement* item) 
{
    ResourceSet::iterator it = resourceSet().begin();
    ResourceSet::iterator end = resourceSet().end();

    for (; it != end; ++it) {
        SVGResource* resource = *it;
        if (!resource->m_clients.contains(item))
            continue;
        resource->m_clients.remove(item);
    }
}

void SVGResource::addClient(SVGStyledElement* item)
{
    if (m_clients.contains(item))
        return;

    m_clients.add(item);
}

TextStream& SVGResource::externalRepresentation(TextStream& ts) const
{
    return ts;
}

SVGResource* getResourceById(Document* document, const AtomicString& id, const RenderObject* object)
{
    if (id.isEmpty())
        return 0;

    Element* element = document->getElementById(id);
    SVGElement* svgElement = 0;
    if (element && element->isSVGElement())
        svgElement = static_cast<SVGElement*>(element);

    if (svgElement && svgElement->isStyled())
        return static_cast<SVGStyledElement*>(svgElement)->canvasResource(object);

    return 0;
}

TextStream& operator<<(TextStream& ts, const SVGResource& r)
{
    return r.externalRepresentation(ts);
}

}

#endif
