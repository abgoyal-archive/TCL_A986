

#ifndef WebInspector_h
#define WebInspector_h

#include "WebKit.h"
#include <wtf/Noncopyable.h>

class WebView;

class WebInspector : public IWebInspector, public IWebInspectorPrivate, public Noncopyable {
public:
    static WebInspector* createInstance(WebView*);

    void webViewClosed();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    virtual HRESULT STDMETHODCALLTYPE show();
    virtual HRESULT STDMETHODCALLTYPE showConsole();
    virtual HRESULT STDMETHODCALLTYPE unused1();
    virtual HRESULT STDMETHODCALLTYPE close();
    virtual HRESULT STDMETHODCALLTYPE attach();
    virtual HRESULT STDMETHODCALLTYPE detach();

    virtual HRESULT STDMETHODCALLTYPE isDebuggingJavaScript(BOOL* isDebugging);
    virtual HRESULT STDMETHODCALLTYPE toggleDebuggingJavaScript();

    virtual HRESULT STDMETHODCALLTYPE isProfilingJavaScript(BOOL* isProfiling);
    virtual HRESULT STDMETHODCALLTYPE toggleProfilingJavaScript();

    virtual HRESULT STDMETHODCALLTYPE isJavaScriptProfilingEnabled(BOOL* isProfilingEnabled);
    virtual HRESULT STDMETHODCALLTYPE setJavaScriptProfilingEnabled(BOOL);

    virtual HRESULT STDMETHODCALLTYPE evaluateInFrontend(ULONG callId, BSTR script);

    virtual HRESULT STDMETHODCALLTYPE isTimelineProfilingEnabled(BOOL* isEnabled);
    virtual HRESULT STDMETHODCALLTYPE setTimelineProfilingEnabled(BOOL);

private:
    WebInspector(WebView*);
    ~WebInspector();

    ULONG m_refCount;
    WebView* m_webView;
};

#endif // !defined(WebInspector_h)
