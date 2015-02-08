

#include "config.h"
#include "GeolocationController.h"

#if ENABLE(CLIENT_BASED_GEOLOCATION)

#include "GeolocationControllerClient.h"

namespace WebCore {

GeolocationController::GeolocationController(Page* page, GeolocationControllerClient* client)
    : m_page(page)
    , m_client(client)
{
}

GeolocationController::~GeolocationController()
{
    if (m_client)
        m_client->geolocationDestroyed();
}

void GeolocationController::addObserver(Geolocation* observer)
{
    ASSERT(!m_observers.contains(observer));

    bool wasEmpty = m_observers.isEmpty();
    m_observers.add(observer);
    if (wasEmpty && m_client)
        m_client->startUpdating();
}

void GeolocationController::removeObserver(Geolocation* observer)
{
    if (!m_observers.contains(observer))
        return;

    m_observers.remove(observer);
    if (m_observers.isEmpty() && m_client)
        m_client->stopUpdating();
}

void GeolocationController::positionChanged(GeolocationPosition* position)
{
    Vector<RefPtr<Geolocation> > observersVector;
    copyToVector(m_observers, observersVector);
    for (size_t i = 0; i < observersVector.size(); ++i)
        observersVector[i]->setPosition(position);
}

void GeolocationController::errorOccurred(GeolocationError* error)
{
    Vector<RefPtr<Geolocation> > observersVector;
    copyToVector(m_observers, observersVector);
    for (size_t i = 0; i < observersVector.size(); ++i)
        observersVector[i]->setError(error);
}

GeolocationPosition* GeolocationController::lastPosition()
{
    if (!m_client)
        return 0;

    return m_client->lastPosition();
}

} // namespace WebCore

#endif // ENABLE(CLIENT_BASED_GEOLOCATION)
