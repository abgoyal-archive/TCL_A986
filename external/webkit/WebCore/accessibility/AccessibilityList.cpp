

#include "config.h"
#include "AccessibilityList.h"

#include "AXObjectCache.h"
#include "HTMLNames.h"
#include "RenderObject.h"

using namespace std;

namespace WebCore {
    
using namespace HTMLNames;

AccessibilityList::AccessibilityList(RenderObject* renderer)
    : AccessibilityRenderObject(renderer)
{
}

AccessibilityList::~AccessibilityList()
{
}

PassRefPtr<AccessibilityList> AccessibilityList::create(RenderObject* renderer)
{
    return adoptRef(new AccessibilityList(renderer));
}

bool AccessibilityList::accessibilityIsIgnored() const
{
    // Is the platform interested in the object?
    if (accessibilityPlatformIncludesObject() == IgnoreObject)
        return true;
    
    // lists don't appear on tiger/leopard on the mac
#if ACCESSIBILITY_LISTS
    return false;
#else
    return true;
#endif
}    
    
bool AccessibilityList::isUnorderedList() const
{
    if (!m_renderer)
        return false;
    
    Node* node = m_renderer->node();

    // The ARIA spec says the "list" role is supposed to mimic a UL or OL tag.
    // Since it can't be both, it's probably OK to say that it's an un-ordered list.
    // On the Mac, there's no distinction to the client.
    if (ariaRoleAttribute() == ListRole)
        return true;
    
    return node && node->hasTagName(ulTag);
}

bool AccessibilityList::isOrderedList() const
{
    if (!m_renderer)
        return false;

    // ARIA says a directory is like a static table of contents, which sounds like an ordered list.
    if (ariaRoleAttribute() == DirectoryRole)
        return true;

    Node* node = m_renderer->node();
    return node && node->hasTagName(olTag);    
}

bool AccessibilityList::isDefinitionList() const
{
    if (!m_renderer)
        return false;
    
    Node* node = m_renderer->node();
    return node && node->hasTagName(dlTag);    
}
    
    
} // namespace WebCore
