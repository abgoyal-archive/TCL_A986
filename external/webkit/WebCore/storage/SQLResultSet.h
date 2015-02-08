

#ifndef SQLResultSet_h
#define SQLResultSet_h

#if ENABLE(DATABASE)

#include "SQLResultSetRowList.h"
#include <wtf/Threading.h>

namespace WebCore {

typedef int ExceptionCode;

class SQLResultSet : public ThreadSafeShared<SQLResultSet> {
public:
    static PassRefPtr<SQLResultSet> create() { return adoptRef(new SQLResultSet); }

    SQLResultSetRowList* rows() const;

    int64_t insertId(ExceptionCode&) const;
    int rowsAffected() const;

// For internal (non-JS) use
    void setInsertId(int64_t);
    void setRowsAffected(int);

private:
    SQLResultSet();

    RefPtr<SQLResultSetRowList> m_rows;
    int64_t m_insertId;
    bool m_insertIdSet;
    int m_rowsAffected;
};

} // namespace WebCore

#endif

#endif // SQLResultSet_h
