

#ifndef GEN_DOMObject_h
#define GEN_DOMObject_h

#include "WebKit.h"

class GEN_DOMObject : public IGEN_DOMObject {
public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

protected:
    GEN_DOMObject();
    virtual ~GEN_DOMObject();

    ULONG m_refCount;
};

#endif // GEN_DOMObject_h
