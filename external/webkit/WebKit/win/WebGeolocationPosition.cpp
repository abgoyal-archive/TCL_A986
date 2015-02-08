

#include "config.h"
#include "WebKitDLL.h"
#include "WebGeolocationPosition.h"
#include <WebCore/COMPtr.h>

#if ENABLE(CLIENT_BASED_GEOLOCATION)
#include <WebCore/GeolocationPosition.h>

using namespace WebCore;
#endif

COMPtr<WebGeolocationPosition> WebGeolocationPosition::createInstance()
{
    return new WebGeolocationPosition;
}

WebGeolocationPosition::WebGeolocationPosition()
    : m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebGeolocationPosition");
}

WebGeolocationPosition::~WebGeolocationPosition()
{
    gClassCount--;
    gClassNameCount.remove("WebGeolocationPosition");
}

HRESULT WebGeolocationPosition::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualIID(riid, __uuidof(WebGeolocationPosition)))
        *ppvObject = this;
    else if (IsEqualIID(riid, __uuidof(IUnknown)))
        *ppvObject = static_cast<IWebGeolocationPosition*>(this);
    else if (IsEqualIID(riid, __uuidof(IWebGeolocationPosition)))
        *ppvObject = static_cast<IWebGeolocationPosition*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG WebGeolocationPosition::AddRef()
{
    return ++m_refCount;
}

ULONG WebGeolocationPosition::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

HRESULT WebGeolocationPosition::initWithTimestamp(double timestamp, double latitude, double longitude, double accuracy)
{
#if ENABLE(CLIENT_BASED_GEOLOCATION)
    m_position = GeolocationPosition::create(timestamp, latitude, longitude, accuracy);
    return S_OK;
#else
    return E_FAIL;
#endif
}

#if ENABLE(CLIENT_BASED_GEOLOCATION)
GeolocationPosition* core(IWebGeolocationPosition* position)
{
    if (!position)
        return 0;

    COMPtr<WebGeolocationPosition> webGeolocationPosition(Query, position);
    if (!webGeolocationPosition)
        return 0;

    return webGeolocationPosition->impl();
}
#endif
