

#ifndef DefaultPolicyDelegate_h
#define DefaultPolicyDelegate_h

#include "COMPtr.h"
#include "WebKit.h"

class DefaultPolicyDelegate : public IWebPolicyDelegate {
public:
    static DefaultPolicyDelegate* sharedInstance();
    static DefaultPolicyDelegate* createInstance();
private:
    DefaultPolicyDelegate();
    ~DefaultPolicyDelegate();
public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebPolicyDelegate
    virtual HRESULT STDMETHODCALLTYPE decidePolicyForNavigationAction( 
        /* [in] */ IWebView *webView,
        /* [in] */ IPropertyBag *actionInformation,
        /* [in] */ IWebURLRequest *request,
        /* [in] */ IWebFrame *frame,
        /* [in] */ IWebPolicyDecisionListener *listener);
    
    virtual HRESULT STDMETHODCALLTYPE decidePolicyForNewWindowAction( 
        /* [in] */ IWebView *webView,
        /* [in] */ IPropertyBag *actionInformation,
        /* [in] */ IWebURLRequest *request,
        /* [in] */ BSTR frameName,
        /* [in] */ IWebPolicyDecisionListener *listener);
    
    virtual HRESULT STDMETHODCALLTYPE decidePolicyForMIMEType( 
        /* [in] */ IWebView *webView,
        /* [in] */ BSTR type,
        /* [in] */ IWebURLRequest *request,
        /* [in] */ IWebFrame *frame,
        /* [in] */ IWebPolicyDecisionListener *listener);
    
    virtual HRESULT STDMETHODCALLTYPE unableToImplementPolicyWithError( 
        /* [in] */ IWebView *webView,
        /* [in] */ IWebError *error,
        /* [in] */ IWebFrame *frame);

protected:
    ULONG m_refCount;
};

#endif // DefaultPolicyDelegate_h
