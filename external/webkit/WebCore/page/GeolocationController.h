

#ifndef GeolocationController_h
#define GeolocationController_h

#if ENABLE(CLIENT_BASED_GEOLOCATION)

#include "Geolocation.h"
#include <wtf/HashSet.h>
#include <wtf/Noncopyable.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class GeolocationControllerClient;
class GeolocationError;
class GeolocationPosition;
class Page;

class GeolocationController : public Noncopyable {
public:
    GeolocationController(Page*, GeolocationControllerClient*);
    ~GeolocationController();

    void addObserver(Geolocation*);
    void removeObserver(Geolocation*);

    void positionChanged(GeolocationPosition*);
    void errorOccurred(GeolocationError*);

    GeolocationPosition* lastPosition();

private:
    Page* m_page;
    GeolocationControllerClient* m_client;

    HashSet<RefPtr<Geolocation> > m_observers;
};

} // namespace WebCore

#endif // ENABLE(CLIENT_BASED_GEOLOCATION)

#endif // GeolocationController_h
