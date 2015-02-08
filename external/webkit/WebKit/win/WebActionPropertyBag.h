

#ifndef WebActionPropertyBag_h
#define WebActionPropertyBag_h

#include "ocidl.h"

#pragma warning(push, 0)
#include <WebCore/Frame.h>
#include <WebCore/HTMLFormElement.h>
#include <WebCore/NavigationAction.h>
#pragma warning(pop)

class WebActionPropertyBag : public IPropertyBag {
public:
    static WebActionPropertyBag* createInstance(const WebCore::NavigationAction&, PassRefPtr<WebCore::HTMLFormElement>, PassRefPtr<WebCore::Frame>);

private:
    WebActionPropertyBag(const WebCore::NavigationAction&, PassRefPtr<WebCore::HTMLFormElement>, PassRefPtr<WebCore::Frame>);
    ~WebActionPropertyBag();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    // IPropertyBag
    virtual /* [local] */ HRESULT STDMETHODCALLTYPE Read( 
        /* [in] */ LPCOLESTR pszPropName,
        /* [out][in] */ VARIANT *pVar,
        /* [in] */ IErrorLog *pErrorLog);
        
    virtual HRESULT STDMETHODCALLTYPE Write( 
        /* [in] */ LPCOLESTR pszPropName,
        /* [in] */ VARIANT *pVar);

private:
    ULONG m_refCount;
    WebCore::NavigationAction m_action;
    RefPtr<WebCore::HTMLFormElement> m_form;
    RefPtr<WebCore::Frame> m_frame;
};

#endif
