


#ifndef WebDropSource_h
#define WebDropSource_h

#include "COMPtr.h"
#include <objidl.h>

class WebView;

namespace WebCore {
    class PlatformMouseEvent;
}

WebCore::PlatformMouseEvent generateMouseEvent(WebView*, bool isDrag);

class WebDropSource : public IDropSource
{
public:
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);        
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);
    virtual HRESULT STDMETHODCALLTYPE QueryContinueDrag(BOOL fEscapePressed, DWORD grfKeyState);
    virtual HRESULT STDMETHODCALLTYPE GiveFeedback(DWORD dwEffect);

    static HRESULT createInstance(WebView* webView, IDropSource** result);
private:
    WebDropSource(WebView* webView);
    ~WebDropSource();
    long m_ref;
    bool m_dropped;
    COMPtr<WebView> m_webView;

};

#endif //!WebDropSource_h
