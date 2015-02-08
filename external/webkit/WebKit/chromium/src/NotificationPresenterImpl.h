

#ifndef NotificationPresenterImpl_h
#define NotificationPresenterImpl_h

#include "NotificationPresenter.h"
#include "VoidCallback.h"

#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>

#if ENABLE(NOTIFICATIONS)

namespace WebKit {

class WebNotificationPresenter;

class NotificationPresenterImpl : public WebCore::NotificationPresenter {
public:
    NotificationPresenterImpl() : m_presenter(0) { }

    void initialize(WebNotificationPresenter* presenter);
    bool isInitialized();

    // WebCore::NotificationPresenter implementation.
    virtual bool show(WebCore::Notification* object);
    virtual void cancel(WebCore::Notification* object);
    virtual void notificationObjectDestroyed(WebCore::Notification* object);
    virtual WebCore::NotificationPresenter::Permission checkPermission(const WebCore::KURL& url, WebCore::Document* document);
    virtual void requestPermission(WebCore::SecurityOrigin* origin, WTF::PassRefPtr<WebCore::VoidCallback> callback);

private:
    // WebNotificationPresenter that this object delegates to.
    WebNotificationPresenter* m_presenter;
};

} // namespace WebKit

#endif // ENABLE(NOTIFICATIONS)

#endif
