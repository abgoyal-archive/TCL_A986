
#include "config.h"
#include "SQLStatement.h"

#if ENABLE(DATABASE)

#include "Database.h"
#include "DatabaseAuthorizer.h"
#include "Logging.h"
#include "SQLError.h"
#include "SQLiteDatabase.h"
#include "SQLiteStatement.h"
#include "SQLResultSet.h"
#include "SQLStatementCallback.h"
#include "SQLStatementErrorCallback.h"
#include "SQLTransaction.h"
#include "SQLValue.h"

namespace WebCore {

PassRefPtr<SQLStatement> SQLStatement::create(const String& statement, const Vector<SQLValue>& arguments, PassRefPtr<SQLStatementCallback> callback, PassRefPtr<SQLStatementErrorCallback> errorCallback, bool readOnly)
{
    return adoptRef(new SQLStatement(statement, arguments, callback, errorCallback, readOnly));
}

SQLStatement::SQLStatement(const String& statement, const Vector<SQLValue>& arguments, PassRefPtr<SQLStatementCallback> callback, PassRefPtr<SQLStatementErrorCallback> errorCallback, bool readOnly)
    : m_statement(statement.crossThreadString())
    , m_arguments(arguments)
    , m_statementCallback(callback)
    , m_statementErrorCallback(errorCallback)
    , m_readOnly(readOnly)
{
}

bool SQLStatement::execute(Database* db)
{
    ASSERT(!m_resultSet);

    // If we're re-running this statement after a quota violation, we need to clear that error now
    clearFailureDueToQuota();

    // This transaction might have been marked bad while it was being set up on the main thread,
    // so if there is still an error, return false.
    if (m_error)
        return false;

    if (m_readOnly)
        db->setAuthorizerReadOnly();

    SQLiteDatabase* database = &db->m_sqliteDatabase;

    SQLiteStatement statement(*database, m_statement);
    int result = statement.prepare();

    if (result != SQLResultOk) {
        LOG(StorageAPI, "Unable to verify correctness of statement %s - error %i (%s)", m_statement.ascii().data(), result, database->lastErrorMsg());
        m_error = SQLError::create(1, database->lastErrorMsg());
        return false;
    }

    // FIXME:  If the statement uses the ?### syntax supported by sqlite, the bind parameter count is very likely off from the number of question marks.
    // If this is the case, they might be trying to do something fishy or malicious
    if (statement.bindParameterCount() != m_arguments.size()) {
        LOG(StorageAPI, "Bind parameter count doesn't match number of question marks");
        m_error = SQLError::create(1, "number of '?'s in statement string does not match argument count");
        return false;
    }

    for (unsigned i = 0; i < m_arguments.size(); ++i) {
        result = statement.bindValue(i + 1, m_arguments[i]);
        if (result == SQLResultFull) {
            setFailureDueToQuota();
            return false;
        }

        if (result != SQLResultOk) {
            LOG(StorageAPI, "Failed to bind value index %i to statement for query '%s'", i + 1, m_statement.ascii().data());
            m_error = SQLError::create(1, database->lastErrorMsg());
            return false;
        }
    }

    RefPtr<SQLResultSet> resultSet = SQLResultSet::create();

    // Step so we can fetch the column names.
    result = statement.step();
    if (result == SQLResultRow) {
        int columnCount = statement.columnCount();
        SQLResultSetRowList* rows = resultSet->rows();

        for (int i = 0; i < columnCount; i++)
            rows->addColumn(statement.getColumnName(i));

        do {
            for (int i = 0; i < columnCount; i++)
                rows->addResult(statement.getColumnValue(i));

            result = statement.step();
        } while (result == SQLResultRow);

        if (result != SQLResultDone) {
            m_error = SQLError::create(1, database->lastErrorMsg());
            return false;
        }
    } else if (result == SQLResultDone) {
        // Didn't find anything, or was an insert
        if (db->m_databaseAuthorizer->lastActionWasInsert())
            resultSet->setInsertId(database->lastInsertRowID());
    } else if (result == SQLResultFull) {
        // Return the Quota error - the delegate will be asked for more space and this statement might be re-run
        setFailureDueToQuota();
        return false;
    } else {
        m_error = SQLError::create(1, database->lastErrorMsg());
        return false;
    }

    // FIXME: If the spec allows triggers, and we want to be "accurate" in a different way, we'd use
    // sqlite3_total_changes() here instead of sqlite3_changed, because that includes rows modified from within a trigger
    // For now, this seems sufficient
    resultSet->setRowsAffected(database->lastChanges());

    m_resultSet = resultSet;
    return true;
}

void SQLStatement::setDatabaseDeletedError()
{
    ASSERT(!m_error && !m_resultSet);
    m_error = SQLError::create(0, "unable to execute statement, because the user deleted the database");
}

void SQLStatement::setVersionMismatchedError()
{
    ASSERT(!m_error && !m_resultSet);
    m_error = SQLError::create(2, "current version of the database and `oldVersion` argument do not match");
}

bool SQLStatement::performCallback(SQLTransaction* transaction)
{
    ASSERT(transaction);

    bool callbackError = false;

    // Call the appropriate statement callback and track if it resulted in an error,
    // because then we need to jump to the transaction error callback.
    if (m_error) {
        ASSERT(m_statementErrorCallback);
        callbackError = m_statementErrorCallback->handleEvent(transaction, m_error.get());
    } else if (m_statementCallback)
        m_statementCallback->handleEvent(transaction, m_resultSet.get(), callbackError);

    // Now release our callbacks, to break reference cycles.
    m_statementCallback = 0;
    m_statementErrorCallback = 0;

    return callbackError;
}

void SQLStatement::setFailureDueToQuota()
{
    ASSERT(!m_error && !m_resultSet);
    m_error = SQLError::create(4, "there was not enough remaining storage space, or the storage quota was reached and the user declined to allow more space");
}

void SQLStatement::clearFailureDueToQuota()
{
    if (lastExecutionFailedDueToQuota())
        m_error = 0;
}

bool SQLStatement::lastExecutionFailedDueToQuota() const
{
    return m_error && m_error->code() == 4;
}

} // namespace WebCore

#endif // ENABLE(DATABASE)
