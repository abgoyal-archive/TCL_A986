

#include "config.h"
#include "WebKitDLL.h"
#include <initguid.h>
#include "WebURLAuthenticationChallengeSender.h"

#include "COMPtr.h"
#include "WebKit.h"
#include "WebURLAuthenticationChallenge.h"
#include "WebURLCredential.h"

#pragma warning(push, 0)
#include <WebCore/AuthenticationClient.h>
#pragma warning(pop)

using namespace WebCore;

// WebURLAuthenticationChallengeSender ----------------------------------------------------------------

WebURLAuthenticationChallengeSender::WebURLAuthenticationChallengeSender(PassRefPtr<AuthenticationClient> client)
    : m_refCount(0)
    , m_client(client)
{
    ASSERT(m_client);
    gClassCount++;
    gClassNameCount.add("WebURLAuthenticationChallengeSender");
}

WebURLAuthenticationChallengeSender::~WebURLAuthenticationChallengeSender()
{
    gClassCount--;
    gClassNameCount.remove("WebURLAuthenticationChallengeSender");
}

WebURLAuthenticationChallengeSender* WebURLAuthenticationChallengeSender::createInstance(PassRefPtr<WebCore::AuthenticationClient> client)
{
    WebURLAuthenticationChallengeSender* instance = new WebURLAuthenticationChallengeSender(client);
    instance->AddRef();
    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IUnknown*>(this);
    else if (IsEqualGUID(riid, __uuidof(this)))
        *ppvObject = static_cast<WebURLAuthenticationChallengeSender*>(this);
    else if (IsEqualGUID(riid, IID_IWebURLAuthenticationChallengeSender))
        *ppvObject = static_cast<IWebURLAuthenticationChallengeSender*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// WebURLAuthenticationChallengeSender ----------------------------------------------------------------

AuthenticationClient* WebURLAuthenticationChallengeSender::authenticationClient() const
{
    return m_client.get();
}

