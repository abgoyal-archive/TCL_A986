

#include "config.h"
#include "AccessibilityMenuListPopup.h"

#include "AXObjectCache.h"
#include "AccessibilityMenuList.h"
#include "AccessibilityMenuListOption.h"
#include "HTMLNames.h"
#include "HTMLSelectElement.h"
#include "RenderObject.h"

namespace WebCore {

using namespace HTMLNames;

AccessibilityMenuListPopup::AccessibilityMenuListPopup()
    : m_menuList(0)
{
}

bool AccessibilityMenuListPopup::isVisible() const
{
    return false;
}

bool AccessibilityMenuListPopup::isOffScreen() const
{
    return m_menuList->isCollapsed();
}

AccessibilityObject* AccessibilityMenuListPopup::parentObject() const
{
    return m_menuList;
}

bool AccessibilityMenuListPopup::isEnabled() const
{
    return m_menuList->isEnabled();
}

AccessibilityMenuListOption* AccessibilityMenuListPopup::menuListOptionAccessibilityObject(HTMLElement* element) const
{
    if (!element || !element->hasTagName(optionTag))
        return 0;

    AccessibilityObject* object = m_menuList->renderer()->document()->axObjectCache()->getOrCreate(MenuListOptionRole);
    ASSERT(object->isMenuListOption());

    AccessibilityMenuListOption* option = static_cast<AccessibilityMenuListOption*>(object);
    option->setElement(element);

    return option;
}

bool AccessibilityMenuListPopup::press() const
{
    m_menuList->press();
    return true;
}

void AccessibilityMenuListPopup::addChildren()
{
    Node* selectNode = m_menuList->renderer()->node();
    if (!selectNode)
        return;

    m_haveChildren = true;

    ASSERT(selectNode->hasTagName(selectTag));

    const Vector<Element*>& listItems = static_cast<HTMLSelectElement*>(selectNode)->listItems();
    unsigned length = listItems.size();
    for (unsigned i = 0; i < length; i++) {
        // The cast to HTMLElement below is safe because the only other possible listItem type
        // would be a WMLElement, but WML builds don't use accessbility features at all.
        AccessibilityMenuListOption* option = menuListOptionAccessibilityObject(static_cast<HTMLElement*>(listItems[i]));
        if (option) {
            option->setParent(this);
            m_children.append(option);
        }
    }
}

void AccessibilityMenuListPopup::childrenChanged()
{
    for (size_t i = m_children.size(); i > 0 ; --i) {
        AccessibilityObject* child = m_children[i - 1].get();
        if (child->actionElement() && !child->actionElement()->attached()) {
            m_menuList->renderer()->document()->axObjectCache()->remove(child->axObjectID());
            m_children.remove(i - 1);
        }
    }
}

void AccessibilityMenuListPopup::setMenuList(AccessibilityMenuList* menuList)
{
    ASSERT_ARG(menuList, menuList);
    ASSERT(!m_menuList);
    m_menuList = menuList;
}

} // namespace WebCore
