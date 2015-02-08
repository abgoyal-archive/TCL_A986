
#ifndef SQLStatement_h
#define SQLStatement_h

#if ENABLE(DATABASE)

#include "PlatformString.h"

#include "SQLError.h"
#include "SQLResultSet.h"
#include "SQLStatementCallback.h"
#include "SQLStatementErrorCallback.h"
#include "SQLValue.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Threading.h>
#include <wtf/Vector.h>

namespace WebCore {

class Database;
class SQLTransaction;
class String;

class SQLStatement : public ThreadSafeShared<SQLStatement> {
public:
    static PassRefPtr<SQLStatement> create(const String&, const Vector<SQLValue>&, PassRefPtr<SQLStatementCallback>, PassRefPtr<SQLStatementErrorCallback>, bool readOnly);

    bool execute(Database*);
    bool lastExecutionFailedDueToQuota() const;

    bool hasStatementCallback() const { return m_statementCallback; }
    bool hasStatementErrorCallback() const { return m_statementErrorCallback; }

    void setDatabaseDeletedError();
    void setVersionMismatchedError();

    bool performCallback(SQLTransaction*);

    SQLError* sqlError() const { return m_error.get(); }
private:
    SQLStatement(const String& statement, const Vector<SQLValue>& arguments, PassRefPtr<SQLStatementCallback> callback, PassRefPtr<SQLStatementErrorCallback> errorCallback, bool readOnly);

    void setFailureDueToQuota();
    void clearFailureDueToQuota();

    String m_statement;
    Vector<SQLValue> m_arguments;
    RefPtr<SQLStatementCallback> m_statementCallback;
    RefPtr<SQLStatementErrorCallback> m_statementErrorCallback;

    RefPtr<SQLError> m_error;
    RefPtr<SQLResultSet> m_resultSet;

    bool m_readOnly;
};

} // namespace WebCore

#endif // ENABLE(DATABASE)

#endif // SQLStatement_h
