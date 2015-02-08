

#include "config.h"
#include "WebKitDLL.h"

#include "WebNotification.h"

#include <wtf/Assertions.h>

// WebNotification ------------------------------------------------------------

WebNotification::WebNotification(BSTR name, IUnknown* anObject, IPropertyBag* userInfo)
: m_refCount(0)
, m_name(0)
, m_anObject(anObject)
, m_userInfo(userInfo)
{
    if (name)
        m_name = SysAllocString(name);
    if (m_anObject)
        m_anObject->AddRef();
    if (m_userInfo)
        m_userInfo->AddRef();

    gClassCount++;
    gClassNameCount.add("WebNotification");
}

WebNotification::~WebNotification()
{
    if (m_name)
        SysFreeString(m_name);
    if (m_anObject)
        m_anObject->Release();
    if (m_userInfo)
        m_userInfo->Release();

    gClassCount--;
    gClassNameCount.remove("WebNotification");
}

WebNotification* WebNotification::createInstance(BSTR name /*=0*/, IUnknown* anObject /*=0*/, IPropertyBag* userInfo /*=0*/)
{
    WebNotification* instance = new WebNotification(name, anObject, userInfo);
    instance->AddRef();
    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebNotification::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebNotification*>(this);
    else if (IsEqualGUID(riid, IID_IWebNotification))
        *ppvObject = static_cast<IWebNotification*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebNotification::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebNotification::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// IWebNotification -----------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebNotification::notificationWithName( 
    /* [in] */ BSTR /*aName*/,
    /* [in] */ IUnknown* /*anObject*/,
    /* [optional][in] */ IPropertyBag* /*userInfo*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebNotification::name( 
    /* [retval][out] */ BSTR* result)
{
    *result = 0;
    if (m_name) {
        *result = SysAllocString(m_name);
        if (!*result)
            return E_OUTOFMEMORY;
    }
    
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebNotification::getObject( 
    /* [retval][out] */ IUnknown** result)
{
    *result = m_anObject;

    if (*result)
        (*result)->AddRef();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebNotification::userInfo( 
    /* [retval][out] */ IPropertyBag** result)
{
    *result = m_userInfo;

    if (*result)
        (*result)->AddRef();

    return S_OK;
}
