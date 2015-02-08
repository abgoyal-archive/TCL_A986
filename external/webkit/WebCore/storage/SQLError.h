

#ifndef SQLError_h
#define SQLError_h

#if ENABLE(DATABASE)

#include "PlatformString.h"
#include <wtf/Threading.h>

namespace WebCore {

class SQLError : public ThreadSafeShared<SQLError> {
public:
    static PassRefPtr<SQLError> create(unsigned code, const String& message) { return adoptRef(new SQLError(code, message)); }

    unsigned code() const { return m_code; }
    String message() const { return m_message.threadsafeCopy(); }

private:
    SQLError(unsigned code, const String& message) : m_code(code), m_message(message.threadsafeCopy()) { }
    unsigned m_code;
    String m_message;
};

}

#endif

#endif // SQLError_h
