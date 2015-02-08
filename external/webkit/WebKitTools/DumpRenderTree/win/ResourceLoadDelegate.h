

#ifndef ResourceLoadDelegate_h
#define ResourceLoadDelegate_h

#include <WebKit/WebKit.h>

class ResourceLoadDelegate : public IWebResourceLoadDelegate {
public:
    ResourceLoadDelegate();
    virtual ~ResourceLoadDelegate();

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebResourceLoadDelegate
    virtual HRESULT STDMETHODCALLTYPE identifierForInitialRequest( 
        /* [in] */ IWebView *webView,
        /* [in] */ IWebURLRequest *request,
        /* [in] */ IWebDataSource *dataSource,
        /* [in] */ unsigned long identifier);
        
    virtual HRESULT STDMETHODCALLTYPE willSendRequest( 
        /* [in] */ IWebView *webView,
        /* [in] */ unsigned long identifier,
        /* [in] */ IWebURLRequest *request,
        /* [in] */ IWebURLResponse *redirectResponse,
        /* [in] */ IWebDataSource *dataSource,
        /* [retval][out] */ IWebURLRequest **newRequest);
        
    virtual HRESULT STDMETHODCALLTYPE didReceiveAuthenticationChallenge( 
        /* [in] */ IWebView *webView,
        /* [in] */ unsigned long identifier,
        /* [in] */ IWebURLAuthenticationChallenge *challenge,
        /* [in] */ IWebDataSource *dataSource);
        
    virtual HRESULT STDMETHODCALLTYPE didCancelAuthenticationChallenge( 
        /* [in] */ IWebView *webView,
        /* [in] */ unsigned long identifier,
        /* [in] */ IWebURLAuthenticationChallenge *challenge,
        /* [in] */ IWebDataSource *dataSource) { return E_NOTIMPL; }
        
    virtual HRESULT STDMETHODCALLTYPE didReceiveResponse( 
        /* [in] */ IWebView *webView,
        /* [in] */ unsigned long identifier,
        /* [in] */ IWebURLResponse *response,
        /* [in] */ IWebDataSource *dataSource);
        
    virtual HRESULT STDMETHODCALLTYPE didReceiveContentLength( 
        /* [in] */ IWebView *webView,
        /* [in] */ unsigned long identifier,
        /* [in] */ UINT length,
        /* [in] */ IWebDataSource *dataSource) { return E_NOTIMPL; }
        
    virtual HRESULT STDMETHODCALLTYPE didFinishLoadingFromDataSource( 
        /* [in] */ IWebView *webView,
        /* [in] */ unsigned long identifier,
        /* [in] */ IWebDataSource *dataSource);
        
    virtual HRESULT STDMETHODCALLTYPE didFailLoadingWithError( 
        /* [in] */ IWebView *webView,
        /* [in] */ unsigned long identifier,
        /* [in] */ IWebError *error,
        /* [in] */ IWebDataSource *dataSource);
        
    virtual HRESULT STDMETHODCALLTYPE plugInFailedWithError( 
        /* [in] */ IWebView *webView,
        /* [in] */ IWebError *error,
        /* [in] */ IWebDataSource *dataSource) { return E_NOTIMPL; }
    
protected:
    ULONG m_refCount;
};

#endif // ResourceLoadDelegate_h
