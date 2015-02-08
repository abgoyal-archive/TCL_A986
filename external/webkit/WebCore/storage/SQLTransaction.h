
#ifndef SQLTransaction_h
#define SQLTransaction_h

#if ENABLE(DATABASE)

#include <wtf/Threading.h>

#include "SQLiteTransaction.h"
#include "SQLStatement.h"
#include "SQLTransactionCallback.h"
#include "SQLTransactionErrorCallback.h"
#include <wtf/Deque.h>
#include <wtf/Forward.h>
#include <wtf/OwnPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

typedef int ExceptionCode;

class Database;
class SQLError;
class SQLStatementCallback;
class SQLStatementErrorCallback;
class SQLTransaction;
class SQLValue;
class String;
class VoidCallback;

class SQLTransactionWrapper : public ThreadSafeShared<SQLTransactionWrapper> {
public:
    virtual ~SQLTransactionWrapper() { }
    virtual bool performPreflight(SQLTransaction*) = 0;
    virtual bool performPostflight(SQLTransaction*) = 0;

    virtual SQLError* sqlError() const = 0;
};

class SQLTransaction : public ThreadSafeShared<SQLTransaction> {
public:
    static PassRefPtr<SQLTransaction> create(Database*, PassRefPtr<SQLTransactionCallback>, PassRefPtr<SQLTransactionErrorCallback>,
                                             PassRefPtr<VoidCallback>, PassRefPtr<SQLTransactionWrapper>, bool readOnly = false);

    ~SQLTransaction();

    void executeSQL(const String& sqlStatement, const Vector<SQLValue>& arguments,
                    PassRefPtr<SQLStatementCallback> callback, PassRefPtr<SQLStatementErrorCallback> callbackError, ExceptionCode& e);

    void lockAcquired();
    bool performNextStep();
    void performPendingCallback();

    Database* database() { return m_database.get(); }
    bool isReadOnly() { return m_readOnly; }
    void notifyDatabaseThreadIsShuttingDown();

private:
    SQLTransaction(Database*, PassRefPtr<SQLTransactionCallback>, PassRefPtr<SQLTransactionErrorCallback>,
                   PassRefPtr<VoidCallback>, PassRefPtr<SQLTransactionWrapper>, bool readOnly);

    typedef void (SQLTransaction::*TransactionStepMethod)();
    TransactionStepMethod m_nextStep;

    void enqueueStatement(PassRefPtr<SQLStatement>);

    void checkAndHandleClosedDatabase();

    void acquireLock();
    void openTransactionAndPreflight();
    void deliverTransactionCallback();
    void scheduleToRunStatements();
    void runStatements();
    void getNextStatement();
    bool runCurrentStatement();
    void handleCurrentStatementError();
    void deliverStatementCallback();
    void deliverQuotaIncreaseCallback();
    void postflightAndCommit();
    void deliverSuccessCallback();
    void cleanupAfterSuccessCallback();
    void handleTransactionError(bool inCallback);
    void deliverTransactionErrorCallback();
    void cleanupAfterTransactionErrorCallback();

#ifndef NDEBUG
    static const char* debugStepName(TransactionStepMethod);
#endif

    RefPtr<SQLStatement> m_currentStatement;

    bool m_executeSqlAllowed;

    RefPtr<Database> m_database;
    RefPtr<SQLTransactionWrapper> m_wrapper;
    RefPtr<SQLTransactionCallback> m_callback;
    RefPtr<VoidCallback> m_successCallback;
    RefPtr<SQLTransactionErrorCallback> m_errorCallback;
    RefPtr<SQLError> m_transactionError;
    bool m_shouldRetryCurrentStatement;
    bool m_modifiedDatabase;
    bool m_lockAcquired;
    bool m_readOnly;

    Mutex m_statementMutex;
    Deque<RefPtr<SQLStatement> > m_statementQueue;

    OwnPtr<SQLiteTransaction> m_sqliteTransaction;
};

} // namespace WebCore

#endif

#endif // SQLTransaction_h
