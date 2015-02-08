

#include "config.h"
#include "WebKitDLL.h"
#include "WebGeolocationControllerClient.h"

#include "WebGeolocationPosition.h"
#include "WebView.h"

using namespace WebCore;

WebGeolocationControllerClient::WebGeolocationControllerClient(WebView* webView)
    : m_webView(webView)
{
}

void WebGeolocationControllerClient::geolocationDestroyed()
{
    delete this;
}

void WebGeolocationControllerClient::startUpdating()
{
    COMPtr<IWebGeolocationProvider> provider;
    if (FAILED(m_webView->geolocationProvider(&provider)))
        return;
    provider->registerWebView(m_webView.get());
}

void WebGeolocationControllerClient::stopUpdating()
{
    COMPtr<IWebGeolocationProvider> provider;
    if (FAILED(m_webView->geolocationProvider(&provider)))
        return;
    provider->registerWebView(m_webView.get());
}

GeolocationPosition* WebGeolocationControllerClient::lastPosition()
{
#if ENABLE(CLIENT_BASED_GEOLOCATION)
    COMPtr<IWebGeolocationProvider> provider;
    if (FAILED(m_webView->geolocationProvider(&provider)))
        return 0;
    COMPtr<IWebGeolocationPosition> position;
    if (FAILED(provider->lastPosition(&position)))
        return 0;
    return core(position.get());
#else
    return 0;
#endif
}
