

#ifndef SQLTransactionClient_h
#define SQLTransactionClient_h

#if ENABLE(DATABASE)

#include <wtf/Noncopyable.h>

namespace WebCore {

    class SQLTransaction;

    // A client to the SQLTransaction class. Allows SQLTransaction to notify interested
    // parties that certain things have happened in a transaction.
    class SQLTransactionClient : public Noncopyable {
    public:
        void didCommitTransaction(SQLTransaction*);
        void didExecuteStatement(SQLTransaction*);
        bool didExceedQuota(SQLTransaction*);
    };
}

#endif // ENABLE(DATABASE)

#endif // SQLTransactionClient_h
