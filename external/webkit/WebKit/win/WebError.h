

#ifndef WebError_h
#define WebError_h

#include "WebKit.h"

#pragma warning(push, 0)
#include <WebCore/COMPtr.h>
#include <WebCore/ResourceError.h>
#pragma warning(pop)

#include <wtf/RetainPtr.h>

class WebError : public IWebError, IWebErrorPrivate {
public:
    static WebError* createInstance(const WebCore::ResourceError&, IPropertyBag* userInfo = 0);
    static WebError* createInstance();
protected:
    WebError(const WebCore::ResourceError&, IPropertyBag* userInfo);
    ~WebError();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebError
    virtual HRESULT STDMETHODCALLTYPE init( 
        /* [in] */ BSTR domain,
        /* [in] */ int code,
        /* [in] */ BSTR url);

    virtual HRESULT STDMETHODCALLTYPE code( 
        /* [retval][out] */ int *result);
        
    virtual HRESULT STDMETHODCALLTYPE domain( 
        /* [retval][out] */ BSTR *result);
        
    virtual HRESULT STDMETHODCALLTYPE localizedDescription( 
        /* [retval][out] */ BSTR *result);
        
    virtual HRESULT STDMETHODCALLTYPE localizedFailureReason( 
        /* [retval][out] */ BSTR *result);
        
    virtual HRESULT STDMETHODCALLTYPE localizedRecoveryOptions( 
        /* [retval][out] */ IEnumVARIANT **result);
        
    virtual HRESULT STDMETHODCALLTYPE localizedRecoverySuggestion( 
        /* [retval][out] */ BSTR *result);
       
    virtual HRESULT STDMETHODCALLTYPE recoverAttempter( 
        /* [retval][out] */ IUnknown **result);
        
    virtual HRESULT STDMETHODCALLTYPE userInfo( 
        /* [retval][out] */ IPropertyBag **result);

    virtual HRESULT STDMETHODCALLTYPE failingURL( 
        /* [retval][out] */ BSTR *result);

    virtual HRESULT STDMETHODCALLTYPE isPolicyChangeError( 
        /* [retval][out] */ BOOL *result);

    // IWebErrorPrivate
    virtual HRESULT STDMETHODCALLTYPE sslPeerCertificate( 
        /* [retval][out] */ OLE_HANDLE *result);

    const WebCore::ResourceError& resourceError() const;

private:
    ULONG m_refCount;
    COMPtr<IPropertyBag> m_userInfo;
    RetainPtr<CFDictionaryRef> m_cfErrorUserInfoDict;
    WebCore::ResourceError m_error;
};

#endif
