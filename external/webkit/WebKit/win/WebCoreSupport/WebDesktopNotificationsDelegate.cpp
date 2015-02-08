

#include "config.h"
#include "WebDesktopNotificationsDelegate.h"
#include "WebSecurityOrigin.h"
#include "WebView.h"
#include <WebCore/BString.h>
#include <WebCore/Document.h>
#include <WebCore/KURL.h>

#if ENABLE(NOTIFICATIONS)

using namespace WebCore;

class NotificationCOMWrapper : public IWebDesktopNotification {
public:
    static NotificationCOMWrapper* create(Notification* inner) { return new NotificationCOMWrapper(inner); }
   
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebDesktopNotification
    HRESULT STDMETHODCALLTYPE isHTML(BOOL* result);
    HRESULT STDMETHODCALLTYPE contentsURL(BSTR* result);
    HRESULT STDMETHODCALLTYPE iconURL(BSTR* result);
    HRESULT STDMETHODCALLTYPE title(BSTR* result);
    HRESULT STDMETHODCALLTYPE text(BSTR* result);
    HRESULT STDMETHODCALLTYPE notifyDisplay();
    HRESULT STDMETHODCALLTYPE notifyError();
    HRESULT STDMETHODCALLTYPE notifyClose(BOOL xplicit);

private:
    NotificationCOMWrapper(Notification* inner) : m_refCount(1), m_inner(inner) {}

    int m_refCount;
    Notification* m_inner;
};

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<NotificationCOMWrapper*>(this);
    else if (IsEqualGUID(riid, IID_IWebDesktopNotification))
        *ppvObject = static_cast<NotificationCOMWrapper*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE NotificationCOMWrapper::AddRef()
{ 
    return ++m_refCount; 
}

ULONG STDMETHODCALLTYPE NotificationCOMWrapper::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);
    return newRef;
}

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::isHTML(BOOL* result)
{
    *result = m_inner->isHTML();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::contentsURL(BSTR* result)
{
    *result = BString(m_inner->url()).release();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::iconURL(BSTR* result)
{
    *result = BString(m_inner->contents().icon()).release();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::title(BSTR* result)
{
    *result = BString(m_inner->contents().title()).release();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::text(BSTR* result)
{
    *result = BString(m_inner->contents().body()).release();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::notifyDisplay()
{
    m_inner->dispatchDisplayEvent();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::notifyError()
{
    m_inner->dispatchErrorEvent();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE NotificationCOMWrapper::notifyClose(BOOL xplicit)
{
    m_inner->dispatchCloseEvent();
    return S_OK;
}

WebDesktopNotificationsDelegate::WebDesktopNotificationsDelegate(WebView* webView)
    : m_webView(webView)
{
}

bool WebDesktopNotificationsDelegate::show(Notification* object)
{
    if (hasNotificationDelegate())
        notificationDelegate()->showDesktopNotification(NotificationCOMWrapper::create(object));
    return true;
}

void WebDesktopNotificationsDelegate::cancel(Notification* object)
{
    if (hasNotificationDelegate())
        notificationDelegate()->cancelDesktopNotification(NotificationCOMWrapper::create(object));
}

void WebDesktopNotificationsDelegate::notificationObjectDestroyed(Notification* object)
{
    if (hasNotificationDelegate())
        notificationDelegate()->notificationDestroyed(NotificationCOMWrapper::create(object));
}

void WebDesktopNotificationsDelegate::requestPermission(SecurityOrigin* origin, PassRefPtr<VoidCallback> callback)
{
    BString org(origin->toString());
    if (hasNotificationDelegate())
        notificationDelegate()->requestNotificationPermission(org);
}

NotificationPresenter::Permission WebDesktopNotificationsDelegate::checkPermission(const KURL& url, Document*)
{
    int out = 0;
    BString org(SecurityOrigin::create(url)->toString());
    if (hasNotificationDelegate())
        notificationDelegate()->checkNotificationPermission(org, &out);
    return (NotificationPresenter::Permission) out;
}

bool WebDesktopNotificationsDelegate::hasNotificationDelegate()
{
    COMPtr<IWebUIDelegate> ui;
    m_webView->uiDelegate(&ui);

    COMPtr<IWebUIDelegate2> ui2;
    return SUCCEEDED(ui->QueryInterface(IID_IWebUIDelegate2, (void**) &ui2));
}

COMPtr<IWebDesktopNotificationsDelegate> WebDesktopNotificationsDelegate::notificationDelegate()
{
    COMPtr<IWebUIDelegate> ui;
    m_webView->uiDelegate(&ui);

    COMPtr<IWebUIDelegate2> ui2;
    COMPtr<IWebDesktopNotificationsDelegate> delegate;
    if (SUCCEEDED(ui->QueryInterface(IID_IWebUIDelegate2, (void**) &ui2)))
        ui2->desktopNotificationsDelegate(&delegate);

    return delegate;
}

#endif  // ENABLE(NOTIFICATIONS)
