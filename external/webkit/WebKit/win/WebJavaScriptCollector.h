

#ifndef WebJavaScriptCollector_H
#define WebJavaScriptCollector_H

#include "WebKit.h"

class WebJavaScriptCollector : public IWebJavaScriptCollector
{
public:
    static WebJavaScriptCollector* createInstance();
protected:
    WebJavaScriptCollector();
    ~WebJavaScriptCollector();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebJavaScriptCollector
    virtual HRESULT STDMETHODCALLTYPE collect();
    
    virtual HRESULT STDMETHODCALLTYPE collectOnAlternateThread( 
        /* [in] */ BOOL waitUntilDone);

    virtual HRESULT STDMETHODCALLTYPE objectCount( 
        /* [retaval][out] */ UINT* count);

protected:
    ULONG m_refCount;
};

#endif
