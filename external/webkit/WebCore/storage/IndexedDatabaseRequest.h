
#ifndef IndexedDatabaseRequest_h
#define IndexedDatabaseRequest_h

#include "ExceptionCode.h"
#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

#if ENABLE(INDEXED_DATABASE)

namespace WebCore {

class IDBRequest;

class IndexedDatabaseRequest : public RefCounted<IndexedDatabaseRequest> {
public:
    static PassRefPtr<IndexedDatabaseRequest> create()
    {
        return adoptRef(new IndexedDatabaseRequest());
    }
    ~IndexedDatabaseRequest();

    IDBRequest* request() const { return m_request.get(); }
    void open(const String& name, const String& description, bool modifyDatabase, ExceptionCode&);

private:
    IndexedDatabaseRequest();

    PassRefPtr<IDBRequest> m_request;
};

} // namespace WebCore

#endif

#endif // IndexedDatabaseRequest_h

