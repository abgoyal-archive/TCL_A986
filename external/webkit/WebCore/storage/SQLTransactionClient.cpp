

#include "config.h"
#include "SQLTransactionClient.h"

#if ENABLE(DATABASE)

#include "Chrome.h"
#include "ChromeClient.h"
#include "Database.h"
#include "DatabaseThread.h"
#include "DatabaseTracker.h"
#include "Document.h"
#include "OriginQuotaManager.h"
#include "Page.h"
#include "SQLTransaction.h"

namespace WebCore {

void SQLTransactionClient::didCommitTransaction(SQLTransaction* transaction)
{
    ASSERT(currentThread() == transaction->database()->scriptExecutionContext()->databaseThread()->getThreadID());
    Database* database = transaction->database();
    DatabaseTracker::tracker().scheduleNotifyDatabaseChanged(
        database->securityOrigin(), database->stringIdentifier());
}

void SQLTransactionClient::didExecuteStatement(SQLTransaction* transaction)
{
    ASSERT(currentThread() == transaction->database()->scriptExecutionContext()->databaseThread()->getThreadID());
    OriginQuotaManager& manager(DatabaseTracker::tracker().originQuotaManager());
    Locker<OriginQuotaManager> locker(manager);
    manager.markDatabase(transaction->database());
}

bool SQLTransactionClient::didExceedQuota(SQLTransaction* transaction)
{
    ASSERT(transaction->database()->scriptExecutionContext()->isContextThread());
    Database* database = transaction->database();

    unsigned long long currentQuota = DatabaseTracker::tracker().quotaForOrigin(database->securityOrigin());
    database->scriptExecutionContext()->databaseExceededQuota(database->stringIdentifier());
    unsigned long long newQuota = DatabaseTracker::tracker().quotaForOrigin(database->securityOrigin());
    return (newQuota > currentQuota);
}

}

#endif // ENABLE(DATABASE)
