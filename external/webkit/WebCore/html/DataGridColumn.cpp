

#include "config.h"

#if ENABLE(DATAGRID)

#include "DataGridColumn.h"

#include "DataGridColumnList.h"

namespace WebCore {

void DataGridColumn::setPrimary(bool primary)
{
    if (m_primary != primary) {
        m_primary = primary;
        if (m_columns)
            m_columns->primaryColumnChanged(this);
    }
}

void DataGridColumn::columnChanged()
{
    if (m_columns)
        m_columns->setDataGridNeedsLayout();
}

} // namespace WebCore

#endif
