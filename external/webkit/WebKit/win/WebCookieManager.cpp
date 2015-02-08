

#include "config.h"
#include "WebKitDLL.h"
#include "WebCookieManager.h"

using namespace WebCore;

// WebCookieManager -------------------------------------------------------

WebCookieManager* WebCookieManager::createInstance()
{
    WebCookieManager* manager = new WebCookieManager;
    manager->AddRef();
    return manager;    
}

WebCookieManager::WebCookieManager()
    : m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebCookieManager");
}

WebCookieManager::~WebCookieManager()
{
    gClassCount--;
    gClassNameCount.remove("WebCookieManager");
}

// IUnknown ---------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebCookieManager::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<WebCookieManager*>(this);
    else if (IsEqualGUID(riid, __uuidof(IWebCookieManager)))
        *ppvObject = static_cast<IWebCookieManager*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebCookieManager::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebCookieManager::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}
