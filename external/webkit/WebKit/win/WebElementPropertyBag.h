

#ifndef WebElementPropertyBag_H
#define WebElementPropertyBag_H

#include "ocidl.h"

#include <WTF/OwnPtr.h>

namespace WebCore {
    class HitTestResult;
}

class WebElementPropertyBag : public IPropertyBag
{
public:
    static WebElementPropertyBag* createInstance(const WebCore::HitTestResult&);

protected:
    WebElementPropertyBag(const WebCore::HitTestResult&);
    ~WebElementPropertyBag();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IPropertyBag
    virtual /* [local] */ HRESULT STDMETHODCALLTYPE Read( 
        /* [in] */ LPCOLESTR pszPropName,
        /* [out][in] */ VARIANT *pVar,
        /* [in] */ IErrorLog *pErrorLog);
        
    virtual HRESULT STDMETHODCALLTYPE Write( 
        /* [in] */ LPCOLESTR pszPropName,
        /* [in] */ VARIANT *pVar);

private:
    OwnPtr<WebCore::HitTestResult> m_result;
    ULONG m_refCount;
};

#endif // WebElementPropertyBag_H
