

#ifndef DataGridColumn_h
#define DataGridColumn_h

#if ENABLE(DATAGRID)

#include "AtomicString.h"
#include "RenderStyle.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class DataGridColumnList;

class DataGridColumn : public RefCounted<DataGridColumn> {
public:
    static PassRefPtr<DataGridColumn> create(const String& columnID, const String& label, const String& type, bool primary, unsigned short sortable)
    {
        return adoptRef(new DataGridColumn(columnID, label, type, primary, sortable));
    }

    const AtomicString& id() const { return m_id; }
    void setId(const AtomicString& id) { m_id = id; columnChanged(); }

    const AtomicString& label() const { return m_label; }
    void setLabel(const AtomicString& label) { m_label = label; columnChanged(); }
    
    const AtomicString& type() const { return m_type; }
    void setType(const AtomicString& type) { m_type = type; columnChanged(); }
    
    unsigned short sortable() const { return m_sortable; }
    void setSortable(unsigned short sortable) { m_sortable = sortable; columnChanged(); }
    
    unsigned short sortDirection() const { return m_sortDirection; }
    void setSortDirection(unsigned short sortDirection) { m_sortDirection = sortDirection; columnChanged(); }
    
    bool primary() const { return m_primary; }
    void setPrimary(bool);

    void setColumnList(DataGridColumnList* list)
    {
        m_columns = list;
        m_columnStyle = 0;
        m_headerStyle = 0;
        m_rect = IntRect();
    }

    RenderStyle* columnStyle() const { return m_columnStyle.get(); }
    void setColumnStyle(PassRefPtr<RenderStyle> style) { m_columnStyle = style; }
    
    RenderStyle* headerStyle() const { return m_headerStyle.get(); }
    void setHeaderStyle(PassRefPtr<RenderStyle> style) { m_headerStyle = style; }
    
    const IntRect& rect() const { return m_rect; }
    void setRect(const IntRect& rect) { m_rect = rect; }

private:
    DataGridColumn(const String& columnID, const String& label, const String& type, bool primary, unsigned short sortable)
        : m_columns(0)
        , m_id(columnID)
        , m_label(label)
        , m_type(type)
        , m_primary(primary)
        , m_sortable(sortable)
        , m_sortDirection(0)
    {
    }

    void columnChanged();

    DataGridColumnList* m_columns; // Not refcounted.  The columns list will null out our reference when it goes away.

    AtomicString m_id;
    AtomicString m_label;
    AtomicString m_type;

    bool m_primary;

    unsigned short m_sortable;
    unsigned short m_sortDirection;
    
    RefPtr<RenderStyle> m_columnStyle; // The style used to render the column background behind the row cells.
    RefPtr<RenderStyle> m_headerStyle; // The style used to render the column header above the row cells.
    
    IntRect m_rect;
};

} // namespace WebCore

#endif

#endif // DataGridColumn_h
