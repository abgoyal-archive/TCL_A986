

#include "config.h"
#include "AccessibilityMenuListOption.h"

#include "AXObjectCache.h"
#include "AccessibilityMenuListPopup.h"
#include "HTMLNames.h"
#include "HTMLOptionElement.h"

namespace WebCore {

using namespace HTMLNames;

AccessibilityMenuListOption::AccessibilityMenuListOption()
    : m_popup(0)
{
}

void AccessibilityMenuListOption::setElement(HTMLElement* element)
{
    ASSERT_ARG(element, element->hasTagName(optionTag));
    m_element = element;
}

Element* AccessibilityMenuListOption::actionElement() const
{
    return m_element.get();
}

AccessibilityObject* AccessibilityMenuListOption::parentObject() const
{
    return m_popup;
}

bool AccessibilityMenuListOption::isEnabled() const
{
    // disabled() returns true if the parent <select> element is disabled,
    // which we don't want.
    return !static_cast<HTMLOptionElement*>(m_element.get())->ownElementDisabled();
}

bool AccessibilityMenuListOption::isVisible() const
{
    // In a single-option select with the popup collapsed, only the selected
    // item is considered visible.
    return !m_popup->isOffScreen() || isSelected();
}

bool AccessibilityMenuListOption::isOffScreen() const
{
    // Invisible list options are considered to be offscreen.
    return !isVisible();
}

bool AccessibilityMenuListOption::isSelected() const
{
    return static_cast<HTMLOptionElement*>(m_element.get())->selected();
}

void AccessibilityMenuListOption::setSelected(bool b)
{
    if (!canSetSelectedAttribute())
        return;

    static_cast<HTMLOptionElement*>(m_element.get())->setSelected(b);
}

String AccessibilityMenuListOption::nameForMSAA() const
{
    return static_cast<HTMLOptionElement*>(m_element.get())->text();
}

bool AccessibilityMenuListOption::canSetSelectedAttribute() const
{
    return isEnabled();
}

IntRect AccessibilityMenuListOption::elementRect() const
{
    AccessibilityObject* parent = parentObject();
    ASSERT(parent->isMenuListPopup());

    AccessibilityObject* grandparent = parent->parentObject();
    ASSERT(grandparent->isMenuList());

    return grandparent->elementRect();
}

} // namespace WebCore
