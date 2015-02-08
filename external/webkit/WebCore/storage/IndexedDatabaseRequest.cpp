
#include "config.h"
#include "IndexedDatabaseRequest.h"

#if ENABLE(INDEXED_DATABASE)

#include "ExceptionCode.h"
#include "IDBRequest.h"

namespace WebCore {

IndexedDatabaseRequest::IndexedDatabaseRequest()
{
}

IndexedDatabaseRequest::~IndexedDatabaseRequest()
{
}

void IndexedDatabaseRequest::open(const String& name, const String& description, bool modifyDatabase, ExceptionCode& exception)
{
}

} // namespace WebCore

#endif // ENABLE(INDEXED_DATABASE)

