

#include "config.h"
#include "AccessibilityTableRow.h"

#include "AXObjectCache.h"
#include "AccessibilityTableCell.h"
#include "HTMLNames.h"
#include "HTMLTableRowElement.h"
#include "RenderObject.h"
#include "RenderTableCell.h"
#include "RenderTableRow.h"

using namespace std;

namespace WebCore {
    
using namespace HTMLNames;
    
AccessibilityTableRow::AccessibilityTableRow(RenderObject* renderer)
    : AccessibilityRenderObject(renderer)
{
}

AccessibilityTableRow::~AccessibilityTableRow()
{
}

PassRefPtr<AccessibilityTableRow> AccessibilityTableRow::create(RenderObject* renderer)
{
    return adoptRef(new AccessibilityTableRow(renderer));
}

AccessibilityRole AccessibilityTableRow::roleValue() const
{
    if (!isTableRow())
        return AccessibilityRenderObject::roleValue();
    
    return RowRole;
}

bool AccessibilityTableRow::isTableRow() const
{
    AccessibilityObject* table = parentTable();
    if (!table || !table->isDataTable())
        return false;
    
    return true;
}
    
bool AccessibilityTableRow::accessibilityIsIgnored() const
{    
    if (!isTableRow())
        return AccessibilityRenderObject::accessibilityIsIgnored();

    return false;
}
    
AccessibilityObject* AccessibilityTableRow::parentTable() const
{
    if (!m_renderer || !m_renderer->isTableRow())
        return 0;
    
    return axObjectCache()->getOrCreate(toRenderTableRow(m_renderer)->table());
}
    
AccessibilityObject* AccessibilityTableRow::headerObject()
{
    if (!m_renderer || !m_renderer->isTableRow())
        return 0;
    
    AccessibilityChildrenVector rowChildren = children();
    if (!rowChildren.size())
        return 0;
    
    // check the first element in the row to see if it is a TH element
    AccessibilityObject* cell = rowChildren[0].get();
    if (!cell->isTableCell())
        return 0;
    
    RenderObject* cellRenderer = static_cast<AccessibilityTableCell*>(cell)->renderer();
    if (!cellRenderer)
        return 0;
    
    Node* cellNode = cellRenderer->node();
    if (!cellNode || !cellNode->hasTagName(thTag))
        return 0;
    
    return cell;
}
    
} // namespace WebCore
