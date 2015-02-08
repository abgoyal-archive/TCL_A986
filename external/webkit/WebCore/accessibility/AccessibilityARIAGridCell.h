

#ifndef AccessibilityARIAGridCell_h
#define AccessibilityARIAGridCell_h

#include "AccessibilityTableCell.h"

namespace WebCore {
    
class AccessibilityARIAGridCell : public AccessibilityTableCell {
    
private:
    AccessibilityARIAGridCell(RenderObject*);
public:
    static PassRefPtr<AccessibilityARIAGridCell> create(RenderObject*);
    virtual ~AccessibilityARIAGridCell();
    
    // fills in the start location and row span of cell
    virtual void rowIndexRange(pair<int, int>& rowRange);
    // fills in the start location and column span of cell
    virtual void columnIndexRange(pair<int, int>& columnRange);
    
protected:
    virtual AccessibilityObject* parentTable() const;
}; 
    
} // namespace WebCore 

#endif // AccessibilityARIAGridCell_h
