

#ifndef WebGeolocationPolicyListener_h
#define WebGeolocationPolicyListener_h

#include "WebKit.h"

#include <WebCore/COMPtr.h>

#include <WTF/PassRefPtr.h>
#include <WTF/RefPtr.h>

namespace WebCore {
    class Geolocation;
}

class WebGeolocationPolicyListener : public IWebGeolocationPolicyListener {
public:
    static COMPtr<WebGeolocationPolicyListener> createInstance(PassRefPtr<WebCore::Geolocation>);

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebGeolocationPolicyListener
    virtual HRESULT STDMETHODCALLTYPE allow();
    virtual HRESULT STDMETHODCALLTYPE deny();

private:
    WebGeolocationPolicyListener(PassRefPtr<WebCore::Geolocation>);
    ~WebGeolocationPolicyListener();

    ULONG m_refCount;
    RefPtr<WebCore::Geolocation> m_geolocation;
};

#endif WebGeolocationPolicyListener_h
