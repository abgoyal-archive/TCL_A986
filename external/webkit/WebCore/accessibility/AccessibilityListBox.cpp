

#include "config.h"
#include "AccessibilityListBox.h"

#include "AXObjectCache.h"
#include "AccessibilityListBoxOption.h"
#include "HTMLNames.h"
#include "HTMLSelectElement.h"
#include "HitTestResult.h"
#include "RenderListBox.h"
#include "RenderObject.h"

using namespace std;

namespace WebCore {

using namespace HTMLNames;

AccessibilityListBox::AccessibilityListBox(RenderObject* renderer)
    : AccessibilityRenderObject(renderer)
{
}

AccessibilityListBox::~AccessibilityListBox()
{
}
    
PassRefPtr<AccessibilityListBox> AccessibilityListBox::create(RenderObject* renderer)
{
    return adoptRef(new AccessibilityListBox(renderer));
}
    
bool AccessibilityListBox::canSetSelectedChildrenAttribute() const
{
    Node* selectNode = m_renderer->node();
    if (!selectNode)
        return false;
    
    return !static_cast<HTMLSelectElement*>(selectNode)->disabled();
}

void AccessibilityListBox::addChildren()
{
    Node* selectNode = m_renderer->node();
    if (!selectNode)
        return;
    
    m_haveChildren = true;
    
    const Vector<Element*>& listItems = static_cast<HTMLSelectElement*>(selectNode)->listItems();
    unsigned length = listItems.size();
    for (unsigned i = 0; i < length; i++) {
        // The cast to HTMLElement below is safe because the only other possible listItem type
        // would be a WMLElement, but WML builds don't use accessibility features at all.
        AccessibilityObject* listOption = listBoxOptionAccessibilityObject(static_cast<HTMLElement*>(listItems[i]));
        if (listOption)
            m_children.append(listOption);
    }
}

void AccessibilityListBox::setSelectedChildren(AccessibilityChildrenVector& children)
{
    if (!canSetSelectedChildrenAttribute())
        return;
    
    Node* selectNode = m_renderer->node();
    if (!selectNode)
        return;
    
    // disable any selected options
    unsigned length = m_children.size();
    for (unsigned i = 0; i < length; i++) {
        AccessibilityListBoxOption* listBoxOption = static_cast<AccessibilityListBoxOption*>(m_children[i].get());
        if (listBoxOption->isSelected())
            listBoxOption->setSelected(false);
    }
    
    length = children.size();
    for (unsigned i = 0; i < length; i++) {
        AccessibilityObject* obj = children[i].get();
        if (obj->roleValue() != ListBoxOptionRole)
            continue;
                
        static_cast<AccessibilityListBoxOption*>(obj)->setSelected(true);
    }
}
    
void AccessibilityListBox::selectedChildren(AccessibilityChildrenVector& result)
{
    ASSERT(result.isEmpty());

    if (!hasChildren())
        addChildren();
        
    unsigned length = m_children.size();
    for (unsigned i = 0; i < length; i++) {
        if (static_cast<AccessibilityListBoxOption*>(m_children[i].get())->isSelected())
            result.append(m_children[i]);
    }    
}

void AccessibilityListBox::visibleChildren(AccessibilityChildrenVector& result)
{
    ASSERT(result.isEmpty());
    
    if (!hasChildren())
        addChildren();
    
    unsigned length = m_children.size();
    for (unsigned i = 0; i < length; i++) {
        if (toRenderListBox(m_renderer)->listIndexIsVisible(i))
            result.append(m_children[i]);
    }
}

AccessibilityObject* AccessibilityListBox::listBoxOptionAccessibilityObject(HTMLElement* element) const
{
    // skip hr elements
    if (!element || element->hasTagName(hrTag))
        return 0;
    
    AccessibilityObject* listBoxObject = m_renderer->document()->axObjectCache()->getOrCreate(ListBoxOptionRole);
    static_cast<AccessibilityListBoxOption*>(listBoxObject)->setHTMLElement(element);
    
    return listBoxObject;
}

AccessibilityObject* AccessibilityListBox::doAccessibilityHitTest(const IntPoint& point) const
{
    // the internal HTMLSelectElement methods for returning a listbox option at a point
    // ignore optgroup elements.
    if (!m_renderer)
        return 0;
    
    Node* node = m_renderer->node();
    if (!node)
        return 0;
    
    IntRect parentRect = boundingBoxRect();
    
    const Vector<Element*>& listItems = static_cast<HTMLSelectElement*>(node)->listItems();
    unsigned length = listItems.size();
    for (unsigned i = 0; i < length; i++) {
        IntRect rect = toRenderListBox(m_renderer)->itemBoundingBoxRect(parentRect.x(), parentRect.y(), i);
        // The cast to HTMLElement below is safe because the only other possible listItem type
        // would be a WMLElement, but WML builds don't use accessibility features at all.
        if (rect.contains(point))
            return listBoxOptionAccessibilityObject(static_cast<HTMLElement*>(listItems[i]));
    }
    
    return axObjectCache()->getOrCreate(m_renderer);
}

} // namespace WebCore
