

#ifndef WebNotificationCenter_H
#define WebNotificationCenter_H

#include "WebKit.h"
#include <wtf/OwnPtr.h>

struct WebNotificationCenterPrivate;

class WebNotificationCenter : public IWebNotificationCenter {
public:
    static WebNotificationCenter* createInstance();

protected:
    WebNotificationCenter();
    ~WebNotificationCenter();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebNotificationCenter
    virtual HRESULT STDMETHODCALLTYPE defaultCenter( 
        /* [retval][out] */ IWebNotificationCenter **center);
    
    virtual HRESULT STDMETHODCALLTYPE addObserver( 
        /* [in] */ IWebNotificationObserver *observer,
        /* [in] */ BSTR notificationName,
        /* [in] */ IUnknown *anObject);
    
    virtual HRESULT STDMETHODCALLTYPE postNotification( 
        /* [in] */ IWebNotification *notification);
    
    virtual HRESULT STDMETHODCALLTYPE postNotificationName( 
        /* [in] */ BSTR notificationName,
        /* [in] */ IUnknown *anObject,
        /* [optional][in] */ IPropertyBag *userInfo);
    
    virtual HRESULT STDMETHODCALLTYPE removeObserver( 
        /* [in] */ IWebNotificationObserver *anObserver,
        /* [in] */ BSTR notificationName,
        /* [optional][in] */ IUnknown *anObject);

    // WebNotificationCenter
    static IWebNotificationCenter* defaultCenterInternal();
    void postNotificationInternal(IWebNotification* notification, BSTR notificationName, IUnknown* anObject);

protected:
    ULONG m_refCount;
    OwnPtr<WebNotificationCenterPrivate> d;
    static IWebNotificationCenter* m_defaultCenter;
};

#endif // WebNotificationCenter_H
