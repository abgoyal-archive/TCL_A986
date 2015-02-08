

#ifndef WebNotification_H
#define WebNotification_H

#include "WebKit.h"

class WebNotification : public IWebNotification
{
public:
    static WebNotification* createInstance(BSTR name = 0, IUnknown* anObject = 0, IPropertyBag* userInfo = 0);
protected:
    WebNotification(BSTR name, IUnknown* anObject, IPropertyBag* userInfo);
    ~WebNotification();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebNotification
    virtual HRESULT STDMETHODCALLTYPE notificationWithName( 
        /* [in] */ BSTR aName,
        /* [in] */ IUnknown *anObject,
        /* [optional][in] */ IPropertyBag *userInfo);
    
    virtual HRESULT STDMETHODCALLTYPE name( 
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE getObject( 
        /* [retval][out] */ IUnknown **result);
    
    virtual HRESULT STDMETHODCALLTYPE userInfo( 
        /* [retval][out] */ IPropertyBag **result);

protected:
    ULONG m_refCount;
    BSTR m_name;
    IUnknown* m_anObject;
    IPropertyBag* m_userInfo;
};

#endif
