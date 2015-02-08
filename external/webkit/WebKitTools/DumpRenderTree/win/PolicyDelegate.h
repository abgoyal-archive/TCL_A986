

#ifndef PolicyDelegate_h
#define PolicyDelegate_h

#include <WebKit/WebKit.h>

class LayoutTestController;

class PolicyDelegate : public IWebPolicyDelegate {
public:
    PolicyDelegate();

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
        /* [in] */ IWebPolicyDecisionListener *listener){ return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE decidePolicyForMIMEType( 
        /* [in] */ IWebView *webView,
        /* [in] */ BSTR type,
        /* [in] */ IWebURLRequest *request,
        /* [in] */ IWebFrame *frame,
        /* [in] */ IWebPolicyDecisionListener *listener){ return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE unableToImplementPolicyWithError( 
        /* [in] */ IWebView *webView,
        /* [in] */ IWebError *error,
        /* [in] */ IWebFrame *frame);

    // PolicyDelegate
    void setPermissive(bool permissive) { m_permissiveDelegate = permissive; }
    void setControllerToNotifyDone(LayoutTestController* controller) { m_controllerToNotifyDone = controller; }

private:
    ULONG m_refCount;
    bool m_permissiveDelegate;
    LayoutTestController* m_controllerToNotifyDone;
};

#endif // PolicyDelegate_h
