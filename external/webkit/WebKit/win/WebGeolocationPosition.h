

#ifndef WebGeolocationPosition_h
#define WebGeolocationPosition_h

#include "WebKit.h"
#include <WebCore/COMPtr.h>

namespace WebCore {
    class GeolocationPosition;
};

class WebGeolocationPosition : public IWebGeolocationPosition {
public:
    static COMPtr<WebGeolocationPosition> createInstance();
private:
    WebGeolocationPosition();
    ~WebGeolocationPosition();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebGeolocationPosition
    virtual HRESULT STDMETHODCALLTYPE initWithTimestamp(double timestamp, double latitude, double longitude, double accuracy);

#if ENABLE(CLIENT_BASED_GEOLOCATION)
    WebCore::GeolocationPosition* impl() const { return m_position.get(); }
#endif

private:
    ULONG m_refCount;
#if ENABLE(CLIENT_BASED_GEOLOCATION)
    RefPtr<WebCore::GeolocationPosition> m_position;
#endif
};

#if ENABLE(CLIENT_BASED_GEOLOCATION)
WebCore::GeolocationPosition* core(IWebGeolocationPosition*);
#endif

#endif // WebGeolocationPosition_h
