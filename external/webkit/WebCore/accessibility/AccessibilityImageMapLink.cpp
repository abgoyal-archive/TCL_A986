

#include "config.h"
#include "AccessibilityImageMapLink.h"

#include "AXObjectCache.h"
#include "AccessibilityRenderObject.h"
#include "Document.h"
#include "HTMLNames.h"
#include "IntRect.h"
#include "RenderObject.h"

using namespace std;

namespace WebCore {
    
using namespace HTMLNames;

AccessibilityImageMapLink::AccessibilityImageMapLink()
    : m_areaElement(0), 
      m_mapElement(0)
{
}

AccessibilityImageMapLink::~AccessibilityImageMapLink()
{
}    

PassRefPtr<AccessibilityImageMapLink> AccessibilityImageMapLink::create()
{
    return adoptRef(new AccessibilityImageMapLink());
}

AccessibilityObject* AccessibilityImageMapLink::parentObject() const
{
    if (m_parent)
        return m_parent;
    
    if (!m_mapElement || !m_mapElement->renderer())
        return 0;
    
    return m_mapElement->document()->axObjectCache()->getOrCreate(m_mapElement->renderer());
}
    
AccessibilityRole AccessibilityImageMapLink::roleValue() const
{
    if (!m_areaElement)
        return WebCoreLinkRole;
    
    const AtomicString& ariaRole = m_areaElement->getAttribute(roleAttr);
    if (!ariaRole.isEmpty())
        return AccessibilityObject::ariaRoleToWebCoreRole(ariaRole);

    return WebCoreLinkRole;
}
    
Element* AccessibilityImageMapLink::actionElement() const
{
    return anchorElement();
}
    
Element* AccessibilityImageMapLink::anchorElement() const
{
    return m_areaElement;
}

KURL AccessibilityImageMapLink::url() const
{
    if (!m_areaElement)
        return KURL();
    
    return m_areaElement->href();
}
    
String AccessibilityImageMapLink::accessibilityDescription() const
{
    if (!m_areaElement)
        return String();

    const AtomicString& alt = m_areaElement->getAttribute(altAttr);
    if (!alt.isEmpty())
        return alt;

    return String();
}
    
String AccessibilityImageMapLink::title() const
{
    if (!m_areaElement)
        return String();
    
    const AtomicString& title = m_areaElement->getAttribute(titleAttr);
    if (!title.isEmpty())
        return title;
    const AtomicString& summary = m_areaElement->getAttribute(summaryAttr);
    if (!summary.isEmpty())
        return summary;

    return String();
}
    
IntRect AccessibilityImageMapLink::elementRect() const
{
    if (!m_mapElement || !m_areaElement)
        return IntRect();

    RenderObject* renderer;
    if (m_parent && m_parent->isAccessibilityRenderObject())
        renderer = static_cast<AccessibilityRenderObject*>(m_parent)->renderer();
    else
        renderer = m_mapElement->renderer();
    
    if (!renderer)
        return IntRect();
    
    return m_areaElement->getRect(renderer);
}
    
IntSize AccessibilityImageMapLink::size() const
{
    return elementRect().size();
}

String AccessibilityImageMapLink::stringValueForMSAA() const
{
    return url();
}

String AccessibilityImageMapLink::nameForMSAA() const
{
    return accessibilityDescription();
}

} // namespace WebCore
