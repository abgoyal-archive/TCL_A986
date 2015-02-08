

#ifndef SQLTransactionCoordinator_h
#define SQLTransactionCoordinator_h

#if ENABLE(DATABASE)

#include "CString.h"
#include "StringHash.h"
#include <wtf/Deque.h>
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class SQLTransaction;

    class SQLTransactionCoordinator : public Noncopyable {
    public:
        void acquireLock(SQLTransaction*);
        void releaseLock(SQLTransaction*);
        void shutdown();
    private:
        typedef Deque<RefPtr<SQLTransaction> > TransactionsQueue;
        struct CoordinationInfo {
            TransactionsQueue pendingTransactions;
            HashSet<RefPtr<SQLTransaction> > activeReadTransactions;
            RefPtr<SQLTransaction> activeWriteTransaction;
        };
        // Maps database names to information about pending transactions
        typedef HashMap<String, CoordinationInfo> CoordinationInfoMap;
        CoordinationInfoMap m_coordinationInfoMap;

        void processPendingTransactions(CoordinationInfo& info);
    };
}

#endif // ENABLE(DATABASE)

#endif // SQLTransactionCoordinator_h
