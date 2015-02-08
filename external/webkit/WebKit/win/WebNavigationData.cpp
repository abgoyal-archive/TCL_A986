

#include "config.h"
#include "WebKitDLL.h"
#include "WebNavigationData.h"

using namespace WebCore;

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebNavigationData::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebNavigationData*>(this);
    else if (IsEqualGUID(riid, IID_IWebNavigationData))
        *ppvObject = static_cast<IWebNavigationData*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebNavigationData::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebNavigationData::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// WebNavigationData -------------------------------------------------------------------

WebNavigationData::WebNavigationData(const String& url, const String& title, IWebURLRequest* request, IWebURLResponse* response, bool hasSubstituteData, const String& clientRedirectSource)
    : m_refCount(0)
    , m_url(url)
    , m_title(title)
    , m_request(request)
    , m_response(response)
    , m_hasSubstituteData(hasSubstituteData)
    , m_clientRedirectSource(clientRedirectSource)

{
    gClassCount++;
    gClassNameCount.add("WebNavigationData");
}

WebNavigationData::~WebNavigationData()
{
    gClassCount--;
    gClassNameCount.remove("WebNavigationData");
}

WebNavigationData* WebNavigationData::createInstance(const String& url, const String& title, IWebURLRequest* request, IWebURLResponse* response, bool hasSubstituteData, const String& clientRedirectSource)
{
    WebNavigationData* instance = new WebNavigationData(url, title, request, response, hasSubstituteData, clientRedirectSource);
    instance->AddRef();
    return instance;
}

// IWebNavigationData -------------------------------------------------------------------

HRESULT WebNavigationData::url(BSTR* url)
{
    if (!url)
        return E_POINTER;
    *url = BString(m_url).release();
    return S_OK;
}

HRESULT WebNavigationData::title(BSTR* title)
{
    if (!title)
        return E_POINTER;
    *title = BString(m_title).release();
    return S_OK;
}

HRESULT WebNavigationData::originalRequest(IWebURLRequest** request)
{
    if (!request)
        return E_POINTER;
    *request = m_request.get();
    m_request->AddRef();
    return S_OK;
}

HRESULT WebNavigationData::response(IWebURLResponse** response)
{
    if (!response)
        return E_POINTER;
    *response = m_response.get();
    m_response->AddRef();
    return S_OK;
}

HRESULT WebNavigationData::hasSubstituteData(BOOL* hasSubstituteData)
{
    if (!hasSubstituteData)
        return E_POINTER;
    *hasSubstituteData = m_hasSubstituteData;
    return S_OK;
}

HRESULT WebNavigationData::clientRedirectSource(BSTR* clientRedirectSource)
{
    if (!clientRedirectSource)
        return E_POINTER;

    *clientRedirectSource = BString(m_clientRedirectSource).release();
    return S_OK;
}
