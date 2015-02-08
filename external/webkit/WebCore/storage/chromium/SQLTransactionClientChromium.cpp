

#include "config.h"
#include "SQLTransactionClient.h"

#include "Database.h"
#include "DatabaseObserver.h"
#include "DatabaseThread.h"
#include "Document.h"
#include "SQLTransaction.h"
#include <wtf/MainThread.h>

namespace WebCore {

class NotifyDatabaseChangedTask : public ScriptExecutionContext::Task {
public:
    static PassOwnPtr<NotifyDatabaseChangedTask> create(Database *database)
    {
        return new NotifyDatabaseChangedTask(database);
    }

    virtual void performTask(ScriptExecutionContext*)
    {
        WebCore::DatabaseObserver::databaseModified(m_database.get());
    }

private:
    NotifyDatabaseChangedTask(PassRefPtr<Database> database)
        : m_database(database)
    {
    }

    RefPtr<Database> m_database;
};

void SQLTransactionClient::didCommitTransaction(SQLTransaction* transaction)
{
    ASSERT(currentThread() == transaction->database()->scriptExecutionContext()->databaseThread()->getThreadID());
    if (!transaction->isReadOnly()) {
        transaction->database()->scriptExecutionContext()->postTask(NotifyDatabaseChangedTask::create(transaction->database()));
    }
}

void SQLTransactionClient::didExecuteStatement(SQLTransaction* transaction)
{
    // This method is called after executing every statement that changes the DB.
    // Chromium doesn't need to do anything at that point.
    ASSERT(currentThread() == transaction->database()->scriptExecutionContext()->databaseThread()->getThreadID());
}

bool SQLTransactionClient::didExceedQuota(SQLTransaction* transaction)
{
    // Chromium does not allow users to manually change the quota for an origin (for now, at least).
    // Don't do anything.
    ASSERT(transaction->database()->scriptExecutionContext()->isContextThread());
    return false;
}

}
