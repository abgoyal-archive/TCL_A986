
#ifndef IDBDatabaseError_h
#define IDBDatabaseError_h

#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

#if ENABLE(INDEXED_DATABASE)

namespace WebCore {

class IDBDatabaseError : public RefCounted<IDBDatabaseError> {
public:
    static PassRefPtr<IDBDatabaseError> create()
    {
        return adoptRef(new IDBDatabaseError());
    }
    ~IDBDatabaseError() { }

    unsigned short code() const { return m_code; }
    void setCode(unsigned short value) { m_code = value; }
    String message() const { return m_message; }
    void setMessage(const String& value) { m_message = value; }

private:
    IDBDatabaseError() { }

    unsigned short m_code;
    String m_message;
};

} // namespace WebCore

#endif

#endif // IDBDatabaseError_h

