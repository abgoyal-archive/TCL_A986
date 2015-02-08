

#ifndef SQLTransactionErrorCallback_h
#define SQLTransactionErrorCallback_h

#if ENABLE(DATABASE)

#include <wtf/Threading.h>

namespace WebCore {

    class SQLError;

    class SQLTransactionErrorCallback : public ThreadSafeShared<SQLTransactionErrorCallback> {
    public:
        virtual ~SQLTransactionErrorCallback() { }
        virtual void handleEvent(SQLError*) = 0;
    };

}

#endif

#endif // SQLTransactionErrorCallback_h
