

#include "config.h"
#include "GEN_DOMObject.h"

#include "WebKitDLL.h"

#include <wtf/Assertions.h>

// DOMObject ------------------------------------------------------------

GEN_DOMObject::GEN_DOMObject()
    : m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("GEN_DOMObject");
}

GEN_DOMObject::~GEN_DOMObject()
{
    gClassCount--;
    gClassNameCount.remove("GEN_DOMObject");
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE GEN_DOMObject::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IGEN_DOMObject*>(this);
    else if (IsEqualGUID(riid, IID_IGEN_DOMObject))
        *ppvObject = static_cast<GEN_DOMObject*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE GEN_DOMObject::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE GEN_DOMObject::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}
