

#ifndef SQLResultSetRowList_h
#define SQLResultSetRowList_h

#if ENABLE(DATABASE)

#include <wtf/PassRefPtr.h>
#include "SQLValue.h"

namespace WebCore {

class SQLResultSetRowList : public RefCounted<SQLResultSetRowList> {
public:
    static PassRefPtr<SQLResultSetRowList> create() { return adoptRef(new SQLResultSetRowList); }

    const Vector<String>& columnNames() const { return m_columns; }
    const Vector<SQLValue>& values() const { return m_result; }

    void addColumn(const String& name) { m_columns.append(name); }
    void addResult(const SQLValue& result) { m_result.append(result); }

    unsigned length() const;

private:
    SQLResultSetRowList() { }

    Vector<String> m_columns;
    Vector<SQLValue> m_result;
};

}

#endif

#endif
