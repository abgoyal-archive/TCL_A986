
#include "config.h"
#include "IDBRequest.h"

#if ENABLE(INDEXED_DATABASE)

#include "IDBDatabaseError.h"
#include "SerializedScriptValue.h"

namespace WebCore {

IDBRequest::IDBRequest(ScriptExecutionContext* context)
    : ActiveDOMObject(context, this)
{
}

IDBRequest::~IDBRequest()
{
}

void IDBRequest::abort()
{
}

EventTargetData* IDBRequest::eventTargetData()
{
    return 0;
}

EventTargetData* IDBRequest::ensureEventTargetData()
{
    return 0;
}

} // namespace WebCore

#endif // ENABLE(INDEXED_DATABASE)

