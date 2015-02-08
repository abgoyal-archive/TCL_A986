

#include "config.h"

#if ENABLE(DATAGRID)

#include "AtomicString.h"
#include "DataGridColumnList.h"
#include "HTMLDataGridElement.h"
#include "PlatformString.h"
#include "RenderObject.h"

namespace WebCore {

DataGridColumnList::DataGridColumnList(HTMLDataGridElement* dataGrid)
    : m_dataGrid(dataGrid)
{
}

DataGridColumnList::~DataGridColumnList()
{
    clear();
}

DataGridColumn* DataGridColumnList::itemWithName(const AtomicString& name) const
{
    unsigned length = m_columns.size();
    for (unsigned i = 0; i < length; ++i) {
        if (m_columns[i]->id() == name)
            return m_columns[i].get();
    }
    return 0;
}

void DataGridColumnList::setDataGridNeedsLayout()
{
    // Mark the datagrid as needing layout.
    if (dataGrid() && dataGrid()->renderer()) 
        dataGrid()->renderer()->setNeedsLayout(true);
}

DataGridColumn* DataGridColumnList::add(const String& id, const String& label, const String& type, bool primary, unsigned short sortable)
{
    return add(DataGridColumn::create(id, label, type, primary, sortable).get());
}

DataGridColumn* DataGridColumnList::add(DataGridColumn* column)
{
    if (column->primary())
        m_primaryColumn = column;
    m_columns.append(column);
    column->setColumnList(this);
    setDataGridNeedsLayout();
    return column;
}

void DataGridColumnList::remove(DataGridColumn* col)
{
    size_t index = m_columns.find(col);
    if (index == notFound)
        return;
    m_columns.remove(index);
    if (col == m_primaryColumn)
        m_primaryColumn = 0;
    if (col == m_sortColumn)
        m_sortColumn = 0;
    col->setColumnList(0);
    setDataGridNeedsLayout();
}

void DataGridColumnList::move(DataGridColumn* col, unsigned long index)
{
    size_t colIndex = m_columns.find(col);
    if (colIndex == notFound)
        return;
    m_columns.insert(index, col);
    setDataGridNeedsLayout();
}

void DataGridColumnList::clear()
{
    unsigned length = m_columns.size();
    for (unsigned i = 0; i < length; ++i)
        m_columns[i]->setColumnList(0);
    m_columns.clear();
    m_primaryColumn = 0;
    m_sortColumn = 0;
    setDataGridNeedsLayout();
}

void DataGridColumnList::primaryColumnChanged(DataGridColumn* col)
{
    if (col->primary())
        m_primaryColumn = col;
    else if (m_primaryColumn = col)
        m_primaryColumn = 0;
    
    setDataGridNeedsLayout();
}

} // namespace WebCore

#endif
