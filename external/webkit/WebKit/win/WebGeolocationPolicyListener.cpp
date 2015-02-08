

#include "config.h"
#include "WebKitDLL.h"
#include "WebGeolocationPolicyListener.h"

#include <WebCore/Geolocation.h>

using namespace WebCore;

// WebGeolocationPolicyListener ----------------------------------------------------------------

COMPtr<WebGeolocationPolicyListener> WebGeolocationPolicyListener::createInstance(PassRefPtr<Geolocation> geolocation)
{
    return new WebGeolocationPolicyListener(geolocation);
}

WebGeolocationPolicyListener::WebGeolocationPolicyListener(PassRefPtr<Geolocation> geolocation)
    : m_refCount(0)
    , m_geolocation(geolocation)
{
    gClassCount++;
    gClassNameCount.add("WebGeolocationPolicyListener");
}

WebGeolocationPolicyListener::~WebGeolocationPolicyListener()
{
    gClassCount--;
    gClassNameCount.remove("WebGeolocationPolicyListener");
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebGeolocationPolicyListener::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualIID(riid, __uuidof(IUnknown)))
        *ppvObject = static_cast<IWebGeolocationPolicyListener*>(this);
    else if (IsEqualIID(riid, __uuidof(IWebGeolocationPolicyListener)))
        *ppvObject = static_cast<IWebGeolocationPolicyListener*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebGeolocationPolicyListener::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebGeolocationPolicyListener::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

// IWebPolicyDecisionListener ------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebGeolocationPolicyListener::allow()
{
    m_geolocation->setIsAllowed(true);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebGeolocationPolicyListener::deny()
{
    m_geolocation->setIsAllowed(false);
    return S_OK;
}
