

#ifndef WebSecurityOrigin_h
#define WebSecurityOrigin_h

#include "WebKit.h"
#include <WebCore/SecurityOrigin.h>

class DECLSPEC_UUID("6EB8D98F-2723-4472-88D3-5936F9D6E631") WebSecurityOrigin : public IWebSecurityOrigin {
public:
    // WebSecurityOrigin
    static WebSecurityOrigin* createInstance(WebCore::SecurityOrigin* origin);
    static WebSecurityOrigin* createInstance(RefPtr<WebCore::SecurityOrigin> origin) { return createInstance(origin.get()); }
    WebCore::SecurityOrigin* securityOrigin() const { return m_securityOrigin.get(); }

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface( 
        /* [in] */ REFIID riid,
        /* [iid_is][out] */ void** ppvObject);

    virtual ULONG STDMETHODCALLTYPE AddRef();

    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebSecurityOrigin
    virtual HRESULT STDMETHODCALLTYPE protocol( 
        /* [retval][out] */ BSTR* result);
        
    virtual HRESULT STDMETHODCALLTYPE host( 
        /* [retval][out] */ BSTR* result);
        
    virtual HRESULT STDMETHODCALLTYPE port( 
        /* [retval][out] */ unsigned short* result);

    virtual HRESULT STDMETHODCALLTYPE usage( 
        /* [retval][out] */ unsigned long long* result);
        
    virtual HRESULT STDMETHODCALLTYPE quota( 
        /* [retval][out] */ unsigned long long* result);
        
    virtual HRESULT STDMETHODCALLTYPE setQuota( 
        /* [in] */ unsigned long long quota);

private:
    WebSecurityOrigin(WebCore::SecurityOrigin*);
    ~WebSecurityOrigin();

    ULONG m_refCount;
    RefPtr<WebCore::SecurityOrigin> m_securityOrigin;
};

#endif // WebSecurityOrigin_h
