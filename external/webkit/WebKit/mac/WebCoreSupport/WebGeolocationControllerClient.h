

#import <WebCore/GeolocationControllerClient.h>

namespace WebCore {
    class GeolocationPosition;
}

@class WebView;

class WebGeolocationControllerClient : public WebCore::GeolocationControllerClient {
public:
    WebGeolocationControllerClient(WebView *);
    WebView *webView() { return m_webView; }

    void geolocationDestroyed();
    void startUpdating();
    void stopUpdating();

    WebCore::GeolocationPosition* lastPosition();

private:
    WebView *m_webView;
};
