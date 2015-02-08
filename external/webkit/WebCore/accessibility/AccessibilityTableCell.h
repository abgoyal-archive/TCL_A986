

#ifndef AccessibilityTableCell_h
#define AccessibilityTableCell_h

#include "AccessibilityRenderObject.h"

namespace WebCore {
    
class AccessibilityTableCell : public AccessibilityRenderObject {
    
protected:
    AccessibilityTableCell(RenderObject*);
public:
    static PassRefPtr<AccessibilityTableCell> create(RenderObject*);
    virtual ~AccessibilityTableCell();
    
    virtual bool isTableCell() const;
    virtual AccessibilityRole roleValue() const;
    
    virtual bool accessibilityIsIgnored() const;

    // fills in the start location and row span of cell
    virtual void rowIndexRange(pair<int, int>& rowRange);
    // fills in the start location and column span of cell
    virtual void columnIndexRange(pair<int, int>& columnRange);
    
    // if a table cell is not exposed as a table cell, a TH element can
    // serve as its title ui element
    AccessibilityObject* titleUIElement() const;
    
protected:
    virtual AccessibilityObject* parentTable() const;
    int m_rowIndex;
}; 
    
} // namespace WebCore 

#endif // AccessibilityTableCell_h
