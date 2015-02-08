

#include "config.h"
#include "DRTDesktopNotificationPresenter.h"

#include "DumpRenderTree.h"
#include "LayoutTestController.h"
#include <JavaScriptCore/JSStringRef.h>
#include <JavaScriptCore/JSStringRefBSTR.h>
#include <WebCore/NotificationPresenter.h>

DRTDesktopNotificationPresenter::DRTDesktopNotificationPresenter()
    : m_refCount(1) {} 

HRESULT STDMETHODCALLTYPE DRTDesktopNotificationPresenter::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<DRTDesktopNotificationPresenter*>(this);
    else if (IsEqualGUID(riid, IID_IWebDesktopNotificationsDelegate))
        *ppvObject = static_cast<DRTDesktopNotificationPresenter*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE DRTDesktopNotificationPresenter::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE DRTDesktopNotificationPresenter::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

HRESULT STDMETHODCALLTYPE DRTDesktopNotificationPresenter::showDesktopNotification(
    /* [in] */ IWebDesktopNotification* notification)
{
    BSTR title, text, url;
    BOOL html;

    if (!notification->isHTML(&html) && html) {
        notification->contentsURL(&url);    
        printf("DESKTOP NOTIFICATION: contents at %S\n", url ? url : L"");
    } else {
        notification->iconURL(&url);
        notification->title(&title);
        notification->text(&text);
        printf("DESKTOP NOTIFICATION: icon %S, title %S, text %S\n", 
            url ? url : L"", 
            title ? title : L"", 
            text ? text : L"");
    }

    // In this stub implementation, the notification is displayed immediately;
    // we dispatch the display event to mimic that.
    notification->notifyDisplay();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE DRTDesktopNotificationPresenter::cancelDesktopNotification(
    /* [in] */ IWebDesktopNotification* notification)
{
    BSTR identifier;
    BOOL html;
    notification->isHTML(&html);
    if (html)
        notification->contentsURL(&identifier);
    else
        notification->title(&identifier);

    printf("DESKTOP NOTIFICATION CLOSED: %S\n", identifier ? identifier : L"");
    notification->notifyClose(false);

    return S_OK;
}

HRESULT STDMETHODCALLTYPE DRTDesktopNotificationPresenter::notificationDestroyed(
    /* [in] */ IWebDesktopNotification* notification)
{
    // Since in these tests events happen immediately, we don't hold on to
    // Notification pointers.  So there's no cleanup to do.
    return S_OK;
}

HRESULT STDMETHODCALLTYPE DRTDesktopNotificationPresenter::checkNotificationPermission(
        /* [in] */ BSTR origin, 
        /* [out, retval] */ int* result)
{
#if ENABLE(NOTIFICATIONS)
    JSStringRef jsOrigin = JSStringCreateWithBSTR(origin);
    bool allowed = ::gLayoutTestController->checkDesktopNotificationPermission(jsOrigin);

    if (allowed)
        *result = WebCore::NotificationPresenter::PermissionAllowed;
    else
        *result = WebCore::NotificationPresenter::PermissionDenied;

    JSStringRelease(jsOrigin);
#endif
    return S_OK;
}

HRESULT STDMETHODCALLTYPE DRTDesktopNotificationPresenter::requestNotificationPermission(
        /* [in] */ BSTR origin)
{
    printf("DESKTOP NOTIFICATION PERMISSION REQUESTED: %S\n", origin ? origin : L"");
    return S_OK;
}
