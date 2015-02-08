

#include "config.h"
#include "WebSecurityOrigin.h"
#include "WebKitDLL.h"

#include <WebCore/BString.h>
#include <WebCore/DatabaseTracker.h>

#if !ENABLE(DATABASE)
#include <wtf/UnusedParam.h>
#endif

using namespace WebCore;

// WebSecurityOrigin ---------------------------------------------------------------
WebSecurityOrigin* WebSecurityOrigin::createInstance(SecurityOrigin* securityOrigin)
{
    WebSecurityOrigin* origin = new WebSecurityOrigin(securityOrigin);
    origin->AddRef();
    return origin;
}

WebSecurityOrigin::WebSecurityOrigin(SecurityOrigin* securityOrigin)
    : m_refCount(0)
    , m_securityOrigin(securityOrigin)
{
    gClassCount++;
    gClassNameCount.add("WebSecurityOrigin");
}

WebSecurityOrigin::~WebSecurityOrigin()
{
    gClassCount--;
    gClassNameCount.remove("WebSecurityOrigin");
}

// IUnknown ------------------------------------------------------------------------
HRESULT STDMETHODCALLTYPE WebSecurityOrigin::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebSecurityOrigin*>(this);
    else if (IsEqualGUID(riid, IID_IWebSecurityOrigin))
        *ppvObject = static_cast<IWebSecurityOrigin*>(this);
    else if (IsEqualGUID(riid, __uuidof(this)))
        *ppvObject = this;
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebSecurityOrigin::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebSecurityOrigin::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

// IWebSecurityOrigin --------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebSecurityOrigin::protocol( 
    /* [retval][out] */ BSTR* result)
{
    if (!result)
        return E_POINTER;

    *result = BString(m_securityOrigin->protocol()).release();

    return S_OK;
}
        
HRESULT STDMETHODCALLTYPE WebSecurityOrigin::host( 
    /* [retval][out] */ BSTR* result)
{
    if (!result)
        return E_POINTER;

    *result = BString(m_securityOrigin->host()).release();

    return S_OK;
}
      
HRESULT STDMETHODCALLTYPE WebSecurityOrigin::port( 
    /* [retval][out] */ unsigned short* result)
{
    if (!result)
        return E_POINTER;

    *result = m_securityOrigin->port();

    return S_OK;
}
        
HRESULT STDMETHODCALLTYPE WebSecurityOrigin::usage( 
    /* [retval][out] */ unsigned long long* result)
{
#if ENABLE(DATABASE)
    if (!result)
        return E_POINTER;

    *result = DatabaseTracker::tracker().usageForOrigin(m_securityOrigin.get());

    return S_OK;
#else
    UNUSED_PARAM(result);
    return E_NOTIMPL;
#endif
}
        
HRESULT STDMETHODCALLTYPE WebSecurityOrigin::quota( 
    /* [retval][out] */ unsigned long long* result)
{
#if ENABLE(DATABASE)
    if (!result)
        return E_POINTER;

    *result = DatabaseTracker::tracker().quotaForOrigin(m_securityOrigin.get());
    return S_OK;
#else
    UNUSED_PARAM(result);
    return E_NOTIMPL;
#endif
}
        
HRESULT STDMETHODCALLTYPE WebSecurityOrigin::setQuota( 
    /* [in] */ unsigned long long quota) 
{
#if ENABLE(DATABASE)
    DatabaseTracker::tracker().setQuota(m_securityOrigin.get(), quota);

    return S_OK;
#else
    UNUSED_PARAM(quota);
    return E_NOTIMPL;
#endif
}
