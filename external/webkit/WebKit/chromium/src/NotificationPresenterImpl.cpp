

#include "config.h"
#include "NotificationPresenterImpl.h"

#if ENABLE(NOTIFICATIONS)

#include "Document.h"
#include "Notification.h"
#include "SecurityOrigin.h"

#include "WebDocument.h"
#include "WebNotification.h"
#include "WebNotificationPermissionCallback.h"
#include "WebNotificationPresenter.h"
#include "WebURL.h"

#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

class VoidCallbackClient : public WebNotificationPermissionCallback {
public:
    VoidCallbackClient(PassRefPtr<VoidCallback> callback)
        : m_callback(callback)
    {
    }

    virtual void permissionRequestComplete()
    {
        if (m_callback)
            m_callback->handleEvent();
        delete this;
    }

private:
    RefPtr<VoidCallback> m_callback;
};

void NotificationPresenterImpl::initialize(WebNotificationPresenter* presenter)
{
    m_presenter = presenter;
}

bool NotificationPresenterImpl::isInitialized()
{
    return !!m_presenter;
}

bool NotificationPresenterImpl::show(Notification* notification)
{
    return m_presenter->show(PassRefPtr<Notification>(notification));
}

void NotificationPresenterImpl::cancel(Notification* notification)
{
    m_presenter->cancel(PassRefPtr<Notification>(notification));
}

void NotificationPresenterImpl::notificationObjectDestroyed(Notification* notification)
{
    m_presenter->objectDestroyed(PassRefPtr<Notification>(notification));
}

NotificationPresenter::Permission NotificationPresenterImpl::checkPermission(const KURL& url, Document* document)
{
    WebDocument webDocument;
    if (document)
        webDocument = document;

    int result = m_presenter->checkPermission(url, document ? &webDocument : 0);
    return static_cast<NotificationPresenter::Permission>(result);
}

void NotificationPresenterImpl::requestPermission(SecurityOrigin* origin, PassRefPtr<VoidCallback> callback)
{
    m_presenter->requestPermission(origin->toString(), new VoidCallbackClient(callback));
}

} // namespace WebKit

#endif // ENABLE(NOTIFICATIONS)
