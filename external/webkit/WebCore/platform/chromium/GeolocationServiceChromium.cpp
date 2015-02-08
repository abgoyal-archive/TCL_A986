

#include "config.h"
#include "GeolocationService.h"

namespace WebCore {

class GeolocationServiceChromium : public GeolocationService {
public:
    GeolocationServiceChromium(GeolocationServiceClient* c)
        : GeolocationService(c)
    {
    }
    // FIXME: Implement. https://bugs.webkit.org/show_bug.cgi?id=32068
};

// This guard is the counterpart of the one in WebCore/platform/GeolocationService.cpp
#if ENABLE(GEOLOCATION)
static GeolocationService* createGeolocationService(GeolocationServiceClient* c)
{
    return new GeolocationServiceChromium(c);
}

GeolocationService::FactoryFunction* GeolocationService::s_factoryFunction = &createGeolocationService;
#endif

} // namespace WebCore
