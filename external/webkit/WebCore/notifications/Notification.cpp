

#include "config.h"

#if ENABLE(NOTIFICATIONS)

#include "Notification.h"
#include "NotificationContents.h"

#include "Document.h"
#include "EventNames.h"
#include "WorkerContext.h"

namespace WebCore {

Notification::Notification(const String& url, ScriptExecutionContext* context, ExceptionCode& ec, NotificationPresenter* provider)
    : ActiveDOMObject(context, this)
    , m_isHTML(true)
    , m_isShowing(false)
    , m_presenter(provider)
{
    ASSERT(m_presenter);
    Document* document = context->isDocument() ? static_cast<Document*>(context) : 0;
    if (m_presenter->checkPermission(context->url(), document) != NotificationPresenter::PermissionAllowed) {
        ec = SECURITY_ERR;
        return;
    }

    m_notificationURL = context->completeURL(url);
    if (url.isEmpty() || !m_notificationURL.isValid()) {
        ec = SYNTAX_ERR;
        return;
    }
}

Notification::Notification(const NotificationContents& contents, ScriptExecutionContext* context, ExceptionCode& ec, NotificationPresenter* provider)
    : ActiveDOMObject(context, this)
    , m_isHTML(false)
    , m_contents(contents)
    , m_isShowing(false)
    , m_presenter(provider)
{
    ASSERT(m_presenter);
    Document* document = context->isDocument() ? static_cast<Document*>(context) : 0;
    if (m_presenter->checkPermission(context->url(), document) != NotificationPresenter::PermissionAllowed) {
        ec = SECURITY_ERR;
        return;
    }
    
    KURL icon = context->completeURL(contents.icon());
    if (!icon.isEmpty() && !icon.isValid()) {
        ec = SYNTAX_ERR;
        return;
    }
}

Notification::~Notification() 
{
    m_presenter->notificationObjectDestroyed(this);
}

void Notification::show() 
{
    // prevent double-showing
    if (!m_isShowing)
        m_isShowing = m_presenter->show(this);
}

void Notification::cancel() 
{
    if (m_isShowing)
        m_presenter->cancel(this);
}

EventTargetData* Notification::eventTargetData()
{
    return &m_eventTargetData;
}

EventTargetData* Notification::ensureEventTargetData()
{
    return &m_eventTargetData;
}

} // namespace WebCore

#endif // ENABLE(NOTIFICATIONS)
