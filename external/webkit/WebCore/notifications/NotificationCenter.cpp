

#include "config.h"

#if ENABLE(NOTIFICATIONS)

#include "NotificationCenter.h"

#include "Document.h"
#include "VoidCallback.h"
#include "WorkerContext.h"

namespace WebCore {

NotificationCenter::NotificationCenter(ScriptExecutionContext* context, NotificationPresenter* presenter)
    : ActiveDOMObject(context, this)
    , m_scriptExecutionContext(context)
    , m_notificationPresenter(presenter) {}

int NotificationCenter::checkPermission()
{
    if (!presenter())
        return NotificationPresenter::PermissionDenied;
    return m_notificationPresenter->checkPermission(
        m_scriptExecutionContext->url(),
        m_scriptExecutionContext->isDocument() ? static_cast<Document*>(m_scriptExecutionContext) : 0);
}

void NotificationCenter::requestPermission(PassRefPtr<VoidCallback> callback)
{
    if (!presenter())
        return;
    m_notificationPresenter->requestPermission(m_scriptExecutionContext->securityOrigin(), callback);
}

} // namespace WebCore

#endif // ENABLE(NOTIFICATIONS)
