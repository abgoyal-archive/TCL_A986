

#ifndef HistoryDelegate_h
#define HistoryDelegate_h

#include <WebKit/WebKit.h>
#include <wtf/OwnPtr.h>

class HistoryDelegate : public IWebHistoryDelegate {
public:
    HistoryDelegate();
    virtual ~HistoryDelegate();

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebHistoryDelegate
    virtual HRESULT STDMETHODCALLTYPE didNavigateWithNavigationData(
        /* [in] */ IWebView* webView, 
        /* [in] */ IWebNavigationData* navigationData, 
        /* [in] */ IWebFrame* webFrame);

    virtual HRESULT STDMETHODCALLTYPE didPerformClientRedirectFromURL(
        /* [in] */ IWebView* webView, 
        /* [in] */ BSTR sourceURL, 
        /* [in] */ BSTR destinationURL, 
        /* [in] */ IWebFrame* webFrame);
    
    virtual HRESULT STDMETHODCALLTYPE didPerformServerRedirectFromURL(
        /* [in] */ IWebView* webView, 
        /* [in] */ BSTR sourceURL, 
        /* [in] */ BSTR destinationURL, 
        /* [in] */ IWebFrame* webFrame);
    
    virtual HRESULT STDMETHODCALLTYPE updateHistoryTitle(
        /* [in] */ IWebView* webView, 
        /* [in] */ BSTR title, 
        /* [in] */ BSTR url);
    
    virtual HRESULT STDMETHODCALLTYPE populateVisitedLinksForWebView(
        /* [in] */ IWebView* webView);

private:
    ULONG m_refCount;
};

#endif // HistoryDelegate_h
