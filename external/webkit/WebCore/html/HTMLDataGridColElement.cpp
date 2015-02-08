

#include "config.h"

#if ENABLE(DATAGRID)

#include "DataGridColumn.h"
#include "HTMLDataGridElement.h"
#include "HTMLDataGridColElement.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "Text.h"

namespace WebCore {

using namespace HTMLNames;

HTMLDataGridColElement::HTMLDataGridColElement(const QualifiedName& name, Document* doc)
    : HTMLElement(name, doc)
    , m_dataGrid(0)
{
}

HTMLDataGridElement* HTMLDataGridColElement::findDataGridAncestor() const
{
    if (parent() && parent()->hasTagName(datagridTag))
        return static_cast<HTMLDataGridElement*>(parent());
    return 0;
}

void HTMLDataGridColElement::ensureColumn()
{
    if (m_column)
        return;
    m_column = DataGridColumn::create(getAttribute(idAttributeName()), label(), type(), primary(), sortable());
}

void HTMLDataGridColElement::insertedIntoTree(bool deep)
{
    HTMLElement::insertedIntoTree(deep);
    if (dataGrid()) // We're connected to a datagrid already.
        return;
    m_dataGrid = findDataGridAncestor();
    if (dataGrid() && dataGrid()->dataSource()->isDOMDataGridDataSource()) {
        ensureColumn();
        m_dataGrid->columns()->add(column()); // FIXME: Deal with ordering issues (complicated, since columns can be made outside the DOM).
    }
}

void HTMLDataGridColElement::removedFromTree(bool deep)
{
    HTMLElement::removedFromTree(deep);
    if (dataGrid() && dataGrid()->dataSource()->isDOMDataGridDataSource()) {
        HTMLDataGridElement* grid = findDataGridAncestor();
        if (!grid && column()) {
            dataGrid()->columns()->remove(column());
            m_dataGrid = 0;
        }
    }
}
    
String HTMLDataGridColElement::label() const
{
    return getAttribute(labelAttr);
}

void HTMLDataGridColElement::setLabel(const String& label)
{
    setAttribute(labelAttr, label);
}

String HTMLDataGridColElement::type() const
{
    return getAttribute(typeAttr);
}

void HTMLDataGridColElement::setType(const String& type)
{
    setAttribute(typeAttr, type);
}

unsigned short HTMLDataGridColElement::sortable() const
{
    if (!hasAttribute(sortableAttr))
        return 2;
    return getAttribute(sortableAttr).toInt(0);
}

void HTMLDataGridColElement::setSortable(unsigned short sortable)
{
    setAttribute(sortableAttr, String::number(sortable));
}

unsigned short HTMLDataGridColElement::sortDirection() const
{
    String sortDirection = getAttribute(sortdirectionAttr);
    if (equalIgnoringCase(sortDirection, "ascending"))
        return 1;
    if (equalIgnoringCase(sortDirection, "descending"))
        return 2;
    return 0;
}

void HTMLDataGridColElement::setSortDirection(unsigned short sortDirection)
{
    // FIXME: Check sortable rules.
    if (sortDirection == 0)
        setAttribute(sortdirectionAttr, "natural");
    else if (sortDirection == 1)
        setAttribute(sortdirectionAttr, "ascending");
    else if (sortDirection == 2)
        setAttribute(sortdirectionAttr, "descending");
}

bool HTMLDataGridColElement::primary() const
{
    return hasAttribute(primaryAttr);
}

void HTMLDataGridColElement::setPrimary(bool primary)
{
    setAttribute(primaryAttr, primary ? "" : 0);
}

void HTMLDataGridColElement::parseMappedAttribute(MappedAttribute* attr) 
{
    HTMLElement::parseMappedAttribute(attr);
     
    if (!column())
        return;

    if (attr->name() == labelAttr)
        column()->setLabel(label());
    else if (attr->name() == typeAttr)
        column()->setType(type());
    else if (attr->name() == primaryAttr)
        column()->setPrimary(primary());
    else if (attr->name() == sortableAttr)
        column()->setSortable(sortable());
    else if (attr->name() == sortdirectionAttr)
        column()->setSortDirection(sortDirection());
    else if (attr->name() == idAttributeName())
        column()->setId(getAttribute(idAttributeName()));
}

} // namespace WebCore

#endif
