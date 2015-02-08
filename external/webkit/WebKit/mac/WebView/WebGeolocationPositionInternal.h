

#import "WebGeolocationPosition.h"

#if ENABLE(CLIENT_BASED_GEOLOCATION)

namespace WebCore {
    class GeolocationPosition;
}

WebCore::GeolocationPosition* core(WebGeolocationPosition *);

#endif
