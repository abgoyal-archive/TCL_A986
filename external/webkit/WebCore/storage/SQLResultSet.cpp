

#include "config.h"
#include "SQLResultSet.h"

#if ENABLE(DATABASE)

#include "ExceptionCode.h"
#include "SQLValue.h"

namespace WebCore {

static unsigned const MaxErrorCode = 2;

SQLResultSet::SQLResultSet()
    : m_rows(SQLResultSetRowList::create())
    , m_insertId(0)
    , m_insertIdSet(false)
    , m_rowsAffected(0)
{
}

int64_t SQLResultSet::insertId(ExceptionCode& e) const
{
    // 4.11.4 - Return the id of the last row inserted as a result of the query
    // If the query didn't result in any rows being added, raise an INVALID_ACCESS_ERR exception
    if (m_insertIdSet)
        return m_insertId;

    e = INVALID_ACCESS_ERR;
    return -1;
}

int SQLResultSet::rowsAffected() const
{
    return m_rowsAffected;
}

SQLResultSetRowList* SQLResultSet::rows() const
{
    return m_rows.get();
}

void SQLResultSet::setInsertId(int64_t id)
{
    ASSERT(!m_insertIdSet);

    m_insertId = id;
    m_insertIdSet = true;
}

void SQLResultSet::setRowsAffected(int count)
{
    m_rowsAffected = count;
}

}

#endif
