

#include "config.h"
#include "WebScriptObject.h"

#include "WebKitDLL.h"

#include <wtf/Assertions.h>

// WebScriptObject ------------------------------------------------------------

WebScriptObject::WebScriptObject()
: m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebScriptObject");
}

WebScriptObject::~WebScriptObject()
{
    gClassCount--;
    gClassNameCount.remove("WebScriptObject");
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebScriptObject::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebScriptObject*>(this);
    else if (IsEqualGUID(riid, IID_IWebScriptObject))
        *ppvObject = static_cast<IWebScriptObject*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebScriptObject::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebScriptObject::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// WebScriptObject ------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebScriptObject::throwException( 
    /* [in] */ BSTR /*exceptionMessage*/,
    /* [retval][out] */ BOOL* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebScriptObject::callWebScriptMethod( 
    /* [in] */ BSTR /*name*/,
    /* [size_is][in] */ const VARIANT /*args*/[  ],
    /* [in] */ int /*cArgs*/,
    /* [retval][out] */ VARIANT* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebScriptObject::evaluateWebScript( 
    /* [in] */ BSTR /*script*/,
    /* [retval][out] */ VARIANT* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebScriptObject::removeWebScriptKey( 
    /* [in] */ BSTR /*name*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebScriptObject::stringRepresentation( 
    /* [retval][out] */ BSTR* /*stringRepresentation*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebScriptObject::webScriptValueAtIndex( 
    /* [in] */ unsigned int /*index*/,
    /* [retval][out] */ VARIANT* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebScriptObject::setWebScriptValueAtIndex( 
    /* [in] */ unsigned int /*index*/,
    /* [in] */ VARIANT /*val*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebScriptObject::setException( 
    /* [in] */ BSTR /*description*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}
