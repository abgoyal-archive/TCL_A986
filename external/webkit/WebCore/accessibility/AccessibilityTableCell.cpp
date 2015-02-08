

#include "config.h"
#include "AccessibilityTableCell.h"

#include "AXObjectCache.h"
#include "HTMLNames.h"
#include "RenderObject.h"
#include "RenderTableCell.h"

using namespace std;

namespace WebCore {
    
using namespace HTMLNames;

AccessibilityTableCell::AccessibilityTableCell(RenderObject* renderer)
    : AccessibilityRenderObject(renderer)
{
}

AccessibilityTableCell::~AccessibilityTableCell()
{
}

PassRefPtr<AccessibilityTableCell> AccessibilityTableCell::create(RenderObject* renderer)
{
    return adoptRef(new AccessibilityTableCell(renderer));
}

bool AccessibilityTableCell::accessibilityIsIgnored() const
{
    if (!isTableCell())
        return AccessibilityRenderObject::accessibilityIsIgnored();
    
    return false;
}
   
AccessibilityObject* AccessibilityTableCell::parentTable() const
{
    if (!m_renderer || !m_renderer->isTableCell())
        return false;
    
    return axObjectCache()->getOrCreate(toRenderTableCell(m_renderer)->table());
}
    
bool AccessibilityTableCell::isTableCell() const
{
    AccessibilityObject* table = parentTable();
    if (!table || !table->isDataTable())
        return false;
    
    return true;
}
    
AccessibilityRole AccessibilityTableCell::roleValue() const
{
    if (!isTableCell())
        return AccessibilityRenderObject::roleValue();
    
    return CellRole;
}
    
void AccessibilityTableCell::rowIndexRange(pair<int, int>& rowRange)
{
    if (!m_renderer || !m_renderer->isTableCell())
        return;
    
    RenderTableCell* renderCell = toRenderTableCell(m_renderer);
    rowRange.first = renderCell->row();
    rowRange.second = renderCell->rowSpan();
    
    // since our table might have multiple sections, we have to offset our row appropriately
    RenderTableSection* section = renderCell->section();
    RenderTable* table = renderCell->table();
    if (!table || !section)
        return;
    
    RenderTableSection* tableSection = table->header();
    if (!tableSection)
        tableSection = table->firstBody();
    
    unsigned rowOffset = 0;
    while (tableSection) {
        if (tableSection == section)
            break;
        rowOffset += tableSection->numRows();
        tableSection = table->sectionBelow(tableSection, true); 
    }

    rowRange.first += rowOffset;
}
    
void AccessibilityTableCell::columnIndexRange(pair<int, int>& columnRange)
{
    if (!m_renderer || !m_renderer->isTableCell())
        return;
    
    RenderTableCell* renderCell = toRenderTableCell(m_renderer);
    columnRange.first = renderCell->col();
    columnRange.second = renderCell->colSpan();    
}
    
AccessibilityObject* AccessibilityTableCell::titleUIElement() const
{
    // Try to find if the first cell in this row is a <th>. If it is,
    // then it can act as the title ui element. (This is only in the
    // case when the table is not appearing as an AXTable.)
    if (isTableCell() || !m_renderer || !m_renderer->isTableCell())
        return 0;

    // Table cells that are th cannot have title ui elements, since by definition
    // they are title ui elements
    Node* node = m_renderer->node();
    if (node && node->hasTagName(thTag))
        return 0;
    
    RenderTableCell* renderCell = toRenderTableCell(m_renderer);

    // If this cell is in the first column, there is no need to continue.
    int col = renderCell->col();
    if (!col)
        return 0;

    int row = renderCell->row();

    RenderTableSection* section = renderCell->section();
    if (!section)
        return 0;
    
    RenderTableCell* headerCell = section->cellAt(row, 0).cell;
    if (!headerCell || headerCell == renderCell)
        return 0;

    Node* cellElement = headerCell->node();
    if (!cellElement || !cellElement->hasTagName(thTag))
        return 0;
    
    return axObjectCache()->getOrCreate(headerCell);
}
    
} // namespace WebCore
