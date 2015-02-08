

#include "config.h"
#include "AccessibilityTableHeaderContainer.h"

#include "AXObjectCache.h"
#include "AccessibilityTable.h"

using namespace std;

namespace WebCore {

AccessibilityTableHeaderContainer::AccessibilityTableHeaderContainer()
    : m_parentTable(0)
{
}

AccessibilityTableHeaderContainer::~AccessibilityTableHeaderContainer()
{
}

PassRefPtr<AccessibilityTableHeaderContainer> AccessibilityTableHeaderContainer::create()
{
    return adoptRef(new AccessibilityTableHeaderContainer());
}
    
const AccessibilityObject::AccessibilityChildrenVector& AccessibilityTableHeaderContainer::children()
{
    if (!m_haveChildren)
        addChildren();
    return m_children;
}
   
IntRect AccessibilityTableHeaderContainer::elementRect() const
{
    // this will be filled in when addChildren is called
    return m_headerRect;
}
    
IntSize AccessibilityTableHeaderContainer::size() const
{
    return elementRect().size();
}
    
void AccessibilityTableHeaderContainer::addChildren()
{
    ASSERT(!m_haveChildren); 
    
    m_haveChildren = true;
    if (!m_parentTable || !m_parentTable->isDataTable())
        return;
    
    static_cast<AccessibilityTable*>(m_parentTable)->columnHeaders(m_children);
    
    unsigned length = m_children.size();
    for (unsigned k = 0; k < length; ++k)
        m_headerRect.unite(m_children[k]->elementRect());
}

} // namespace WebCore
