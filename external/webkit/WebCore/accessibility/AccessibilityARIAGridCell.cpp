

#include "config.h"
#include "AccessibilityARIAGridCell.h"

#include "AccessibilityObject.h"
#include "AccessibilityTableRow.h"

using namespace std;

namespace WebCore {
    
AccessibilityARIAGridCell::AccessibilityARIAGridCell(RenderObject* renderer)
    : AccessibilityTableCell(renderer)
{
}

AccessibilityARIAGridCell::~AccessibilityARIAGridCell()
{
}

PassRefPtr<AccessibilityARIAGridCell> AccessibilityARIAGridCell::create(RenderObject* renderer)
{
    return adoptRef(new AccessibilityARIAGridCell(renderer));
}

AccessibilityObject* AccessibilityARIAGridCell::parentTable() const
{
    AccessibilityObject* parent = parentObjectUnignored();
    if (!parent || !parent->isTableRow())
        return 0;
    
    parent = parent->parentObjectUnignored();
    if (!parent || !parent->isDataTable())
        return 0;
    
    return parent;
}
    
void AccessibilityARIAGridCell::rowIndexRange(pair<int, int>& rowRange)
{
    AccessibilityObject* parent = parentObjectUnignored();
    if (!parent || !parent->isTableRow())
        return;

    // as far as I can tell, grid cells cannot span rows
    rowRange.first = static_cast<AccessibilityTableRow*>(parent)->rowIndex();
    rowRange.second = 1;    
}

void AccessibilityARIAGridCell::columnIndexRange(pair<int, int>& columnRange)
{
    AccessibilityObject* parent = parentObjectUnignored();
    if (!parent || !parent->isTableRow())
        return;
    
    AccessibilityChildrenVector siblings = parent->children();
    unsigned childrenSize = siblings.size();
    for (unsigned k = 0; k < childrenSize; ++k) {
        if (siblings[k].get() == this) {
            columnRange.first = k;
            break;
        }
    }
    
    // as far as I can tell, grid cells cannot span columns
    columnRange.second = 1;    
}
  
} // namespace WebCore
