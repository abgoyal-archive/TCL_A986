

#include "config.h"
#include "SQLResultSetRowList.h"

#if ENABLE(DATABASE)

namespace WebCore {

unsigned SQLResultSetRowList::length() const
{
    if (m_result.size() == 0)
        return 0;

    ASSERT(m_result.size() % m_columns.size() == 0);

    return m_result.size() / m_columns.size();
}

}

#endif
