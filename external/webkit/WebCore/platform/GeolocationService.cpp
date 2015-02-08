

#include "config.h"
#include "GeolocationService.h"
#include "Geoposition.h"
#include "GeolocationServiceMock.h"
#include "PositionError.h"

#include <wtf/CurrentTime.h>
#include <wtf/Assertions.h>

namespace WebCore {

#if !ENABLE(GEOLOCATION) || ENABLE(CLIENT_BASED_GEOLOCATION)
static GeolocationService* createGeolocationServiceNull(GeolocationServiceClient*)
{
    return 0;
}

GeolocationService::FactoryFunction* GeolocationService::s_factoryFunction = &createGeolocationServiceNull;
#endif

GeolocationService* GeolocationService::create(GeolocationServiceClient* client)
{
    return (*s_factoryFunction)(client);
}

void GeolocationService::useMock()
{
    s_factoryFunction = &GeolocationServiceMock::create;
}

GeolocationService::GeolocationService(GeolocationServiceClient* client)
    : m_geolocationServiceClient(client)
{
    ASSERT(m_geolocationServiceClient);
}

void GeolocationService::positionChanged()
{
    m_geolocationServiceClient->geolocationServicePositionChanged(this);
}

void GeolocationService::errorOccurred()
{
    m_geolocationServiceClient->geolocationServiceErrorOccurred(this);
}

} // namespace WebCore
