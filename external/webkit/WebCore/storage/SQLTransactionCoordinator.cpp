

#include "config.h"
#include "SQLTransactionCoordinator.h"

#if ENABLE(DATABASE)

#include "CString.h"
#include "Database.h"
#include "SQLTransaction.h"
#include <wtf/Deque.h>
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/RefPtr.h>

namespace WebCore {

static String getDatabaseIdentifier(SQLTransaction* transaction)
{
    Database* database = transaction->database();
    ASSERT(database);
    return database->stringIdentifier();
}

void SQLTransactionCoordinator::processPendingTransactions(CoordinationInfo& info)
{
    if (info.activeWriteTransaction || info.pendingTransactions.isEmpty())
        return;

    RefPtr<SQLTransaction> firstPendingTransaction = info.pendingTransactions.first();
    if (firstPendingTransaction->isReadOnly()) {
        do {
            firstPendingTransaction = info.pendingTransactions.first();
            info.pendingTransactions.removeFirst();
            info.activeReadTransactions.add(firstPendingTransaction);
            firstPendingTransaction->lockAcquired();
        } while (!info.pendingTransactions.isEmpty() && info.pendingTransactions.first()->isReadOnly());
    } else if (info.activeReadTransactions.isEmpty()) {
        info.pendingTransactions.removeFirst();
        info.activeWriteTransaction = firstPendingTransaction;
        firstPendingTransaction->lockAcquired();
    }
}

void SQLTransactionCoordinator::acquireLock(SQLTransaction* transaction)
{
    String dbIdentifier = getDatabaseIdentifier(transaction);

    CoordinationInfoMap::iterator coordinationInfoIterator = m_coordinationInfoMap.find(dbIdentifier);
    if (coordinationInfoIterator == m_coordinationInfoMap.end()) {
        // No pending transactions for this DB
        coordinationInfoIterator = m_coordinationInfoMap.add(dbIdentifier, CoordinationInfo()).first;
    }

    CoordinationInfo& info = coordinationInfoIterator->second;
    info.pendingTransactions.append(transaction);
    processPendingTransactions(info);
}

void SQLTransactionCoordinator::releaseLock(SQLTransaction* transaction)
{
    if (m_coordinationInfoMap.isEmpty())
        return;

    String dbIdentifier = getDatabaseIdentifier(transaction);

    CoordinationInfoMap::iterator coordinationInfoIterator = m_coordinationInfoMap.find(dbIdentifier);
    ASSERT(coordinationInfoIterator != m_coordinationInfoMap.end());
    CoordinationInfo& info = coordinationInfoIterator->second;

    if (transaction->isReadOnly()) {
        ASSERT(info.activeReadTransactions.contains(transaction));
        info.activeReadTransactions.remove(transaction);
    } else {
        ASSERT(info.activeWriteTransaction == transaction);
        info.activeWriteTransaction = 0;
    }

    processPendingTransactions(info);
}

void SQLTransactionCoordinator::shutdown()
{
    // Notify all transactions in progress that the database thread is shutting down
    for (CoordinationInfoMap::iterator coordinationInfoIterator = m_coordinationInfoMap.begin();
         coordinationInfoIterator != m_coordinationInfoMap.end(); ++coordinationInfoIterator) {
        CoordinationInfo& info = coordinationInfoIterator->second;
        if (info.activeWriteTransaction)
            info.activeWriteTransaction->notifyDatabaseThreadIsShuttingDown();
        for (HashSet<RefPtr<SQLTransaction> >::iterator activeReadTransactionsIterator =
                     info.activeReadTransactions.begin();
             activeReadTransactionsIterator != info.activeReadTransactions.end();
             ++activeReadTransactionsIterator) {
            (*activeReadTransactionsIterator)->notifyDatabaseThreadIsShuttingDown();
        }
    }

    // Clean up all pending transactions for all databases
    m_coordinationInfoMap.clear();
}

} // namespace WebCore

#endif // ENABLE(DATABASE)
