

#ifndef HTMLDataGridElement_h
#define HTMLDataGridElement_h

#if ENABLE(DATAGRID)

#include "DataGridColumnList.h"
#include "DataGridDataSource.h"
#include "HTMLElement.h"
#include "Timer.h"

namespace WebCore {

class HTMLDataGridElement : public HTMLElement {
public:
    HTMLDataGridElement(const QualifiedName&, Document*);
    virtual ~HTMLDataGridElement();

    virtual int tagPriority() const { return 6; } // Same as <select>s
    virtual bool checkDTD(const Node*);

    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

    bool autofocus() const;
    void setAutofocus(bool);

    bool disabled() const;
    void setDisabled(bool);

    bool multiple() const;
    void setMultiple(bool);

    void setDataSource(PassRefPtr<DataGridDataSource>);
    DataGridDataSource* dataSource() const;

    DataGridColumnList* columns() const { return m_columns.get(); }

private:
    RefPtr<DataGridDataSource> m_dataSource;
    RefPtr<DataGridColumnList> m_columns;
};

} // namespace WebCore

#endif

#endif // HTMLDataGridElement_h
