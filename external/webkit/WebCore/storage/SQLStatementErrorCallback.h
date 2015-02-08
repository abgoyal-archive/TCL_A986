

#ifndef SQLStatementErrorCallback_h
#define SQLStatementErrorCallback_h

#if ENABLE(DATABASE)

#include <wtf/Threading.h>

namespace WebCore {

class SQLTransaction;
class SQLError;

class SQLStatementErrorCallback : public ThreadSafeShared<SQLStatementErrorCallback> {
public:
    virtual ~SQLStatementErrorCallback() { }
    virtual bool handleEvent(SQLTransaction*, SQLError*) = 0;
};

}

#endif

#endif // SQLStatementErrorCallback_h
