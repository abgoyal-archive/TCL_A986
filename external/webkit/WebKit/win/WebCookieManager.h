

#ifndef WebCookieManager_h
#define WebCookieManager_h

#include "WebKit.h"

typedef struct OpaqueCFHTTPCookieStorage*  CFHTTPCookieStorageRef;

class WebCookieManager : public IWebCookieManager {
public:
    static WebCookieManager* createInstance();

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface( 
        /* [in] */ REFIID riid,
        /* [iid_is][out] */ void** ppvObject);

    virtual ULONG STDMETHODCALLTYPE AddRef();

    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebCookieManager
    virtual /* [local] */ HRESULT STDMETHODCALLTYPE cookieStorage( 
        /* [retval][out] */ CFHTTPCookieStorageRef* storage);

    virtual /* [local] */ HRESULT STDMETHODCALLTYPE setCookieStorage( 
        /* [in] */ CFHTTPCookieStorageRef storage);

private:
    WebCookieManager();
    ~WebCookieManager();

    ULONG m_refCount;
};

#endif
