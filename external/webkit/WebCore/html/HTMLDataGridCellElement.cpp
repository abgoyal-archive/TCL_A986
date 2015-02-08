

#include "config.h"

#if ENABLE(DATAGRID)

#include "HTMLDataGridCellElement.h"

#include "HTMLNames.h"
#include "Text.h"

namespace WebCore {

using namespace HTMLNames;

HTMLDataGridCellElement::HTMLDataGridCellElement(const QualifiedName& name, Document* doc)
    : HTMLElement(name, doc)
{
}

String HTMLDataGridCellElement::label() const
{
    return getAttribute(labelAttr);
}

void HTMLDataGridCellElement::setLabel(const String& label)
{
    setAttribute(labelAttr, label);
}

bool HTMLDataGridCellElement::focused() const
{
    return hasAttribute(focusedAttr);
}

void HTMLDataGridCellElement::setFocused(bool focused)
{
    setAttribute(focusedAttr, focused ? "" : 0);
}

bool HTMLDataGridCellElement::checked() const
{
    return hasAttribute(checkedAttr);
}

void HTMLDataGridCellElement::setChecked(bool checked)
{
    setAttribute(checkedAttr, checked ? "" : 0);
}

bool HTMLDataGridCellElement::indeterminate() const
{
    return hasAttribute(indeterminateAttr);
}

void HTMLDataGridCellElement::setIndeterminate(bool indeterminate)
{
    setAttribute(indeterminateAttr, indeterminate ? "" : 0);
}

float HTMLDataGridCellElement::progress() const
{
    return getAttribute(progressAttr).toInt();
}

void HTMLDataGridCellElement::setProgress(float progress)
{
    setAttribute(progressAttr, String::number(progress));
}

} // namespace WebCore

#endif
