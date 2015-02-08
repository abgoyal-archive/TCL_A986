

#include "config.h"

#if ENABLE(DATAGRID)

#include "HTMLDataGridRowElement.h"

#include "HTMLNames.h"
#include "Text.h"

namespace WebCore {

using namespace HTMLNames;

HTMLDataGridRowElement::HTMLDataGridRowElement(const QualifiedName& name, Document* doc)
    : HTMLElement(name, doc)
{
}

bool HTMLDataGridRowElement::checkDTD(const Node* newChild)
{
    if (newChild->isTextNode())
        return static_cast<const Text*>(newChild)->containsOnlyWhitespace();
    return newChild->hasTagName(drowTag) || newChild->hasTagName(dcellTag);
}

bool HTMLDataGridRowElement::selected() const
{
    return hasAttribute(selectedAttr);
}

void HTMLDataGridRowElement::setSelected(bool selected)
{
    setAttribute(selectedAttr, selected ? "" : 0);
}

bool HTMLDataGridRowElement::focused() const
{
    return hasAttribute(focusedAttr);
}

void HTMLDataGridRowElement::setFocused(bool focused)
{
    setAttribute(focusedAttr, focused ? "" : 0);
}

bool HTMLDataGridRowElement::expanded() const
{
    return hasAttribute(expandedAttr);
}

void HTMLDataGridRowElement::setExpanded(bool expanded)
{
    setAttribute(expandedAttr, expanded ? "" : 0);
}

} // namespace WebCore

#endif
