

#ifndef HTMLDataGridColElement_h
#define HTMLDataGridColElement_h

#if ENABLE(DATAGRID)

#include "DataGridColumn.h"
#include "HTMLElement.h"

namespace WebCore {

class HTMLDataGridElement;

class HTMLDataGridColElement : public HTMLElement {
public:
    HTMLDataGridColElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }
    virtual void insertedIntoTree(bool /*deep*/);
    virtual void removedFromTree(bool /*deep*/);
    virtual void parseMappedAttribute(MappedAttribute*);

    String label() const;
    void setLabel(const String&);
    
    String type() const;
    void setType(const String&);
    
    unsigned short sortable() const;
    void setSortable(unsigned short);
    
    unsigned short sortDirection() const;
    void setSortDirection(unsigned short);
    
    bool primary() const;
    void setPrimary(bool);
    
    DataGridColumn* column() const { return m_column.get(); }
    void setColumn(PassRefPtr<DataGridColumn> col) { m_column = col; }

private:
    HTMLDataGridElement* dataGrid() const { return m_dataGrid; }
    HTMLDataGridElement* findDataGridAncestor() const;
    void ensureColumn();

    RefPtr<DataGridColumn> m_column;
    HTMLDataGridElement* m_dataGrid; // Not refcounted. We will null out our reference if we get removed from the grid.
};

} // namespace WebCore

#endif

#endif // HTMLDataGridColElement_h
