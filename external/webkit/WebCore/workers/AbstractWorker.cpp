

#include "config.h"

#if ENABLE(WORKERS)

#include "AbstractWorker.h"

#include "ErrorEvent.h"
#include "Event.h"
#include "EventException.h"
#include "EventNames.h"
#include "ScriptExecutionContext.h"
#include "SecurityOrigin.h"

namespace WebCore {

AbstractWorker::AbstractWorker(ScriptExecutionContext* context)
    : ActiveDOMObject(context, this)
{
}

AbstractWorker::~AbstractWorker()
{
}

KURL AbstractWorker::resolveURL(const String& url, ExceptionCode& ec)
{
    if (url.isEmpty()) {
        ec = SYNTAX_ERR;
        return KURL();
    }

    // FIXME: This should use the dynamic global scope (bug #27887)
    KURL scriptURL = scriptExecutionContext()->completeURL(url);
    if (!scriptURL.isValid()) {
        ec = SYNTAX_ERR;
        return KURL();
    }

    if (!scriptExecutionContext()->securityOrigin()->canAccess(SecurityOrigin::create(scriptURL).get())) {
        ec = SECURITY_ERR;
        return KURL();
    }
    return scriptURL;
}

EventTargetData* AbstractWorker::eventTargetData()
{
    return &m_eventTargetData;
}

EventTargetData* AbstractWorker::ensureEventTargetData()
{
    return &m_eventTargetData;
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
