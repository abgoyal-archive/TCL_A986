

#ifndef AccessibilityTable_h
#define AccessibilityTable_h

#include "AccessibilityRenderObject.h"

#if PLATFORM(MAC) && (defined(BUILDING_ON_TIGER) || defined(BUILDING_ON_LEOPARD))
#define ACCESSIBILITY_TABLES 0
#else
#define ACCESSIBILITY_TABLES 1
#endif

namespace WebCore {

class String;
class AccessibilityTableCell;
class AccessibilityTableHeaderContainer;
    
class AccessibilityTable : public AccessibilityRenderObject {

protected:
    AccessibilityTable(RenderObject*);
public:
    static PassRefPtr<AccessibilityTable> create(RenderObject*);
    virtual ~AccessibilityTable();
    
    virtual bool isDataTable() const;
    virtual AccessibilityRole roleValue() const;
    virtual bool isAriaTable() const { return false; }
    
    virtual bool accessibilityIsIgnored() const;
    
    virtual void addChildren();
    virtual void clearChildren();
    
    AccessibilityChildrenVector& columns();
    AccessibilityChildrenVector& rows();
    
    virtual bool supportsSelectedRows() { return false; }
    unsigned columnCount();
    unsigned rowCount();
    
    virtual String title() const;
    
    // all the cells in the table
    void cells(AccessibilityChildrenVector&);
    virtual AccessibilityTableCell* cellForColumnAndRow(unsigned column, unsigned row);
    
    void columnHeaders(AccessibilityChildrenVector&);
    void rowHeaders(AccessibilityChildrenVector&);

    // an object that contains, as children, all the objects that act as headers
    AccessibilityObject* headerContainer();
    
protected:    
    AccessibilityChildrenVector m_rows;
    AccessibilityChildrenVector m_columns;
    
    AccessibilityTableHeaderContainer* m_headerContainer;
    mutable bool m_isAccessibilityTable;
    
    bool isTableExposableThroughAccessibility();
};
    
} // namespace WebCore 

#endif // AccessibilityTable_h
