

#ifndef AccessibilityTableColumn_h
#define AccessibilityTableColumn_h

#include "AccessibilityObject.h"
#include "AccessibilityTable.h"
#include "IntRect.h"

namespace WebCore {
    
class RenderTableSection;

class AccessibilityTableColumn : public AccessibilityObject {
    
private:
    AccessibilityTableColumn();
public:
    static PassRefPtr<AccessibilityTableColumn> create();
    virtual ~AccessibilityTableColumn();
    
    void setParentTable(AccessibilityTable*);
    virtual AccessibilityObject* parentObject() const { return m_parentTable; }
    AccessibilityObject* headerObject();
        
    virtual AccessibilityRole roleValue() const { return ColumnRole; }
    virtual bool accessibilityIsIgnored() const { return false; }
    virtual bool isTableColumn() const { return true; }
    
    void setColumnIndex(int columnIndex) { m_columnIndex = columnIndex; }
    int columnIndex() const { return m_columnIndex; }    
    
    virtual const AccessibilityChildrenVector& children();
    virtual void addChildren();
    
    virtual IntSize size() const;
    virtual IntRect elementRect() const;
    
private:    
    AccessibilityTable* m_parentTable;
    int m_columnIndex;
    IntRect m_columnRect;
    
    AccessibilityObject* headerObjectForSection(RenderTableSection*, bool thTagRequired);
};
   
} // namespace WebCore 

#endif // AccessibilityTableColumn_h
