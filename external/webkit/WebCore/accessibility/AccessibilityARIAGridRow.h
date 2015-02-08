

#ifndef AccessibilityARIAGridRow_h
#define AccessibilityARIAGridRow_h

#include "AccessibilityTableRow.h"

namespace WebCore {
    
class AccessibilityARIAGridRow : public AccessibilityTableRow {
    
private:
    AccessibilityARIAGridRow(RenderObject*);
public:
    static PassRefPtr<AccessibilityARIAGridRow> create(RenderObject*);
    virtual ~AccessibilityARIAGridRow();
    
    void disclosedRows(AccessibilityChildrenVector&);
    AccessibilityObject* disclosedByRow() const;
    
    virtual AccessibilityObject* headerObject();
    virtual AccessibilityObject* parentTable() const;    
    
private:
    virtual bool isARIATreeGridRow() const;
}; 
    
} // namespace WebCore 

#endif // AccessibilityARIAGridRow_h
