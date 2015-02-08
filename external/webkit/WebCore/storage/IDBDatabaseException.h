
#ifndef IDBDatabaseException_h
#define IDBDatabaseException_h

#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

#if ENABLE(INDEXED_DATABASE)

namespace WebCore {

class IDBDatabaseException : public RefCounted<IDBDatabaseException> {
public:
    static PassRefPtr<IDBDatabaseException> create()
    {
        return adoptRef(new IDBDatabaseException());
    }
    ~IDBDatabaseException() { }
    
    unsigned short code() const { return m_code; }
    void setCode(unsigned short value) { m_code = value; }
    String message() const { return m_message; }
    void setMessage(const String& value) { m_message = value; }

private:
    IDBDatabaseException() { }

    unsigned short m_code;
    String m_message;
};

} // namespace WebCore

#endif

#endif // IDBDatabaseException_h

