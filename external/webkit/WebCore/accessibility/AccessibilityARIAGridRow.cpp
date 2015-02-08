

#include "config.h"
#include "AccessibilityARIAGridRow.h"

#include "AccessibilityObject.h"
#include "AccessibilityTable.h"
#include "RenderObject.h"

using namespace std;

namespace WebCore {
    
AccessibilityARIAGridRow::AccessibilityARIAGridRow(RenderObject* renderer)
    : AccessibilityTableRow(renderer)
{
}

AccessibilityARIAGridRow::~AccessibilityARIAGridRow()
{
}

PassRefPtr<AccessibilityARIAGridRow> AccessibilityARIAGridRow::create(RenderObject* renderer)
{
    return adoptRef(new AccessibilityARIAGridRow(renderer));
}

bool AccessibilityARIAGridRow::isARIATreeGridRow() const
{
    AccessibilityObject* parent = parentTable();
    if (!parent)
        return false;
    
    return parent->ariaRoleAttribute() == TreeGridRole;
}
    
void AccessibilityARIAGridRow::disclosedRows(AccessibilityChildrenVector& disclosedRows)
{
    // The contiguous disclosed rows will be the rows in the table that 
    // have an aria-level of plus 1 from this row.
    AccessibilityObject* parent = parentObjectUnignored();
    if (!parent || !parent->isDataTable())
        return;
    
    // Search for rows that match the correct level. 
    // Only take the subsequent rows from this one that are +1 from this row's level.
    int index = rowIndex();
    if (index < 0)
        return;
    
    unsigned level = hierarchicalLevel();
    AccessibilityChildrenVector& allRows = static_cast<AccessibilityTable*>(parent)->rows();
    int rowCount = allRows.size();
    for (int k = index + 1; k < rowCount; ++k) {
        AccessibilityObject* row = allRows[k].get();
        // Stop at the first row that doesn't match the correct level.
        if (row->hierarchicalLevel() != level + 1)
            break;

        disclosedRows.append(row);
    }
}
    
AccessibilityObject* AccessibilityARIAGridRow::disclosedByRow() const
{
    // The row that discloses this one is the row in the table
    // that is aria-level subtract 1 from this row.
    AccessibilityObject* parent = parentObjectUnignored();
    if (!parent || !parent->isDataTable())
        return 0;
    
    // If the level is 1 or less, than nothing discloses this row.
    unsigned level = hierarchicalLevel();
    if (level <= 1)
        return 0;
    
    // Search for the previous row that matches the correct level.
    int index = rowIndex();
    AccessibilityChildrenVector& allRows = static_cast<AccessibilityTable*>(parent)->rows();
    int rowCount = allRows.size();
    if (index >= rowCount)
        return 0;
    
    for (int k = index - 1; k >= 0; --k) {
        AccessibilityObject* row = allRows[k].get();
        if (row->hierarchicalLevel() == level - 1)
            return row;
    }
    
    return 0;
}
    
AccessibilityObject* AccessibilityARIAGridRow::parentTable() const
{
    AccessibilityObject* parent = parentObjectUnignored();
    if (!parent->isDataTable())
        return 0;
    
    return parent;
}

AccessibilityObject* AccessibilityARIAGridRow::headerObject()
{
    AccessibilityChildrenVector rowChildren = children();
    unsigned childrenCount = rowChildren.size();
    for (unsigned i = 0; i < childrenCount; ++i) {
        AccessibilityObject* cell = rowChildren[i].get();
        if (cell->ariaRoleAttribute() == RowHeaderRole)
            return cell;
    }
    
    return 0;
}

} // namespace WebCore
