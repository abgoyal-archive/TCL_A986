

#include "config.h"
#include "GeolocationServiceMock.h"

#include "Logging.h"
#include "Geolocation.h"
#include "Geoposition.h"
#include "PositionError.h"
#include "PositionOptions.h"

namespace WebCore {

GeolocationServiceMock::GeolocationServiceSet* GeolocationServiceMock::s_instances = 0;
RefPtr<Geoposition>* GeolocationServiceMock::s_lastPosition;
RefPtr<PositionError>* GeolocationServiceMock::s_lastError;

GeolocationService* GeolocationServiceMock::create(GeolocationServiceClient* client)
{
    initStatics();
    return new GeolocationServiceMock(client);
}

GeolocationServiceMock::GeolocationServiceMock(GeolocationServiceClient* client)
    : GeolocationService(client)
    , m_timer(this, &GeolocationServiceMock::timerFired)
    , m_isActive(false)
{
    s_instances->add(this);
}

GeolocationServiceMock::~GeolocationServiceMock()
{
    GeolocationServiceSet::iterator iter = s_instances->find(this);
    ASSERT(iter != s_instances->end());
    s_instances->remove(iter);
    cleanUpStatics();
}

void GeolocationServiceMock::setPosition(PassRefPtr<Geoposition> position)
{
    initStatics();
    GeolocationService::useMock();
    *s_lastPosition = position;
    *s_lastError = 0;
    makeGeolocationCallbackFromAllInstances();
}

void GeolocationServiceMock::setError(PassRefPtr<PositionError> error)
{
    initStatics();
    GeolocationService::useMock();
    *s_lastError = error;
    *s_lastPosition = 0;
    makeGeolocationCallbackFromAllInstances();
}

#if PLATFORM(ANDROID)
// TODO: Upstream to webkit.org. See https://bugs.webkit.org/show_bug.cgi?id=34082
bool GeolocationServiceMock::startUpdating(PositionOptions*, bool /* suspend */)
#else
bool GeolocationServiceMock::startUpdating(PositionOptions*)
#endif
{
    m_isActive = true;
    m_timer.startOneShot(0);
    return true;
}

void GeolocationServiceMock::stopUpdating()
{
    m_isActive = false;
}

void GeolocationServiceMock::timerFired(Timer<GeolocationServiceMock>* timer)
{
    ASSERT_UNUSED(timer, timer == &m_timer);
    makeGeolocationCallback();
}

void GeolocationServiceMock::makeGeolocationCallbackFromAllInstances()
{
    GeolocationServiceSet::const_iterator end = s_instances->end();
    for (GeolocationServiceSet::const_iterator iter = s_instances->begin(); iter != end; ++iter)
        (*iter)->makeGeolocationCallback();
}

void GeolocationServiceMock::makeGeolocationCallback()
{
    if (!m_isActive)
        return;

    if (*s_lastPosition)
        positionChanged();
    else if (*s_lastError)
        errorOccurred();
}

void GeolocationServiceMock::initStatics()
{
    if (s_instances == 0) { 
        s_instances = new GeolocationServiceSet;
        s_lastPosition = new RefPtr<Geoposition>;
        s_lastError = new RefPtr<PositionError>;
    }
}

void GeolocationServiceMock::cleanUpStatics()
{
    if (s_instances->size() == 0) {
        delete s_instances;
        s_instances = 0;
        delete s_lastPosition;
        delete s_lastError;
    }
}

} // namespace WebCore
