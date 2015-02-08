

#include <WebCore/COMPtr.h>
#include <WebCore/Notification.h>
#include <WebCore/NotificationPresenter.h>

#if ENABLE(NOTIFICATIONS)

interface IWebDesktopNotificationPresenter;

namespace WebCore {
class Document;
class KURL;
}

class WebDesktopNotificationsDelegate : public WebCore::NotificationPresenter {
public:
    WebDesktopNotificationsDelegate(WebView* view);

    /* WebCore::NotificationPresenter interface */
    virtual bool show(WebCore::Notification* object);
    virtual void cancel(WebCore::Notification* object);
    virtual void notificationObjectDestroyed(WebCore::Notification* object);
    virtual void requestPermission(WebCore::SecurityOrigin* origin, PassRefPtr<WebCore::VoidCallback> callback);
    virtual WebCore::NotificationPresenter::Permission checkPermission(const KURL& url, Document* document);

private:
    bool hasNotificationDelegate();
    COMPtr<IWebDesktopNotificationsDelegate> notificationDelegate();

    WebView* m_webView;
};

#endif
