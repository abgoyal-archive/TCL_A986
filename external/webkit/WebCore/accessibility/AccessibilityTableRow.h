

#ifndef AccessibilityTableRow_h
#define AccessibilityTableRow_h

#include "AccessibilityRenderObject.h"

namespace WebCore {
    
class AccessibilityTableRow : public AccessibilityRenderObject {
    
protected:
    AccessibilityTableRow(RenderObject*);
public:
    static PassRefPtr<AccessibilityTableRow> create(RenderObject*);
    virtual ~AccessibilityTableRow();
    
    virtual bool isTableRow() const;
    virtual AccessibilityRole roleValue() const;
    virtual bool accessibilityIsIgnored() const;

    // retrieves the "row" header (a th tag in the rightmost column)
    virtual AccessibilityObject* headerObject();
    virtual AccessibilityObject* parentTable() const;
    
    void setRowIndex(int rowIndex) { m_rowIndex = rowIndex; }
    int rowIndex() const { return m_rowIndex; }

    // allows the table to add other children that may not originate
    // in the row, but their col/row spans overlap into it
    void appendChild(AccessibilityObject*);
    
private:
    int m_rowIndex;
}; 
   
} // namespace WebCore 

#endif // AccessibilityTableRow_h
