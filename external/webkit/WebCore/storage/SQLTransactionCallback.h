

#ifndef SQLTransactionCallback_h
#define SQLTransactionCallback_h

#if ENABLE(DATABASE)

#include <wtf/Threading.h>

namespace WebCore {

class SQLTransaction;
class SQLError;

class SQLTransactionCallback : public ThreadSafeShared<SQLTransactionCallback> {
public:
    virtual ~SQLTransactionCallback() { }
    virtual void handleEvent(SQLTransaction*, bool& raisedException) = 0;
};

}

#endif

#endif // SQLTransactionCallback_h
