

#ifndef AccessibilityTableHeaderContainer_h
#define AccessibilityTableHeaderContainer_h

#include "AccessibilityObject.h"
#include "AccessibilityTable.h"
#include "IntRect.h"

namespace WebCore {

class AccessibilityTableHeaderContainer : public AccessibilityObject {
    
private:
    AccessibilityTableHeaderContainer();
public:
    static PassRefPtr<AccessibilityTableHeaderContainer> create();
    virtual ~AccessibilityTableHeaderContainer();
    
    virtual AccessibilityRole roleValue() const { return TableHeaderContainerRole; }
    
    void setParentTable(AccessibilityTable* table) { m_parentTable = table; }
    virtual AccessibilityObject* parentObject() const { return m_parentTable; }
    
    virtual bool accessibilityIsIgnored() const { return false; }
    
    virtual const AccessibilityChildrenVector& children();
    virtual void addChildren();
    
    virtual IntSize size() const;
    virtual IntRect elementRect() const;
    
private:
    AccessibilityTable* m_parentTable;
    IntRect m_headerRect;
    
}; 
    
} // namespace WebCore 

#endif // AccessibilityTableHeaderContainer_h
