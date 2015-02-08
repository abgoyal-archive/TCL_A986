
#ifndef SQLStatementCallback_h
#define SQLStatementCallback_h

#if ENABLE(DATABASE)

#include <wtf/Threading.h>

namespace WebCore {

class SQLTransaction;
class SQLResultSet;

class SQLStatementCallback : public ThreadSafeShared<SQLStatementCallback> {
public:
    virtual ~SQLStatementCallback() { }
    virtual void handleEvent(SQLTransaction*, SQLResultSet*, bool& raisedException) = 0;
};

}

#endif

#endif // SQLStatementErrorCallback_h
