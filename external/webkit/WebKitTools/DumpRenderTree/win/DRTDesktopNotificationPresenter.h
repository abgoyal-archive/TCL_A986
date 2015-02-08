

#ifndef DRTDesktopNotificationPresenter_h
#define DRTDesktopNotificationPresenter_h

#include <WebKit/WebKit.h>
#include <wtf/OwnPtr.h>
#include <windef.h>

class DRTDesktopNotificationPresenter : public IWebDesktopNotificationsDelegate {
public:
    DRTDesktopNotificationPresenter();

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebDesktopNotificationsDelegate
    virtual HRESULT STDMETHODCALLTYPE showDesktopNotification(
        /* [in] */ IWebDesktopNotification* notification);

    virtual HRESULT STDMETHODCALLTYPE cancelDesktopNotification(
        /* [in] */ IWebDesktopNotification* notification);

    virtual HRESULT STDMETHODCALLTYPE notificationDestroyed(
        /* [in] */ IWebDesktopNotification* notification);

    virtual HRESULT STDMETHODCALLTYPE checkNotificationPermission(
        /* [in] */ BSTR origin, 
        /* [out, retval] */ int* result);

    virtual HRESULT STDMETHODCALLTYPE requestNotificationPermission(
        /* [in] */ BSTR origin);

private:
    ULONG m_refCount;
};

#endif
