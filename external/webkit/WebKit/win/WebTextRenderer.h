

#ifndef WebTextRenderer_h
#define WebTextRenderer_h

#include "WebKit.h"

class WebTextRenderer : public IWebTextRenderer {
public:
    static WebTextRenderer* createInstance();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(
        /* [in] */ REFIID riid,
        /* [iid_is][out] */ void** ppvObject);
    
    virtual ULONG STDMETHODCALLTYPE AddRef();
    
    virtual ULONG STDMETHODCALLTYPE Release();

    virtual HRESULT STDMETHODCALLTYPE registerPrivateFont(
        /* [in] */ LPCOLESTR fontFilePath);
    
private:
    WebTextRenderer();
    ~WebTextRenderer();

    ULONG m_refCount;
};

#endif // !defined(WebTextRenderer_h)
