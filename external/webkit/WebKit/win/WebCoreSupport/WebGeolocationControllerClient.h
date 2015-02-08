

#ifndef WebGeolocationControllerClient_h
#define WebGeolocationControllerClient_h

#include "COMPtr.h"
#include <WebCore/GeolocationControllerClient.h>

namespace WebCore {
    class GeolocationPosition;
}

class WebView;

class WebGeolocationControllerClient : public WebCore::GeolocationControllerClient {
public:
    WebGeolocationControllerClient(WebView*);

    virtual void geolocationDestroyed();
    virtual void startUpdating();
    virtual void stopUpdating();
    virtual WebCore::GeolocationPosition* lastPosition();

private:
    COMPtr<WebView> m_webView;
};

#endif // WebGeolocationControllerClient_h
