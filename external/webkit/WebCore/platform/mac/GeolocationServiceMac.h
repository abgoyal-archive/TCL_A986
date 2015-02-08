

#ifndef GeolocationServiceMac_h
#define GeolocationServiceMac_h

#if ENABLE(GEOLOCATION)

#include "GeolocationService.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/RetainPtr.h>

#ifdef __OBJC__
@class CLLocationManager;
@class WebCoreCoreLocationObserver;
#else
class CLLocationManager;
class WebCoreCoreLocationObserver;
#endif

namespace WebCore {

class GeolocationServiceMac : public GeolocationService {
public:
    static GeolocationService* create(GeolocationServiceClient*);
    virtual ~GeolocationServiceMac();
    
    virtual bool startUpdating(PositionOptions*);
    virtual void stopUpdating();

    virtual void suspend();
    virtual void resume();

    virtual Geoposition* lastPosition() const { return m_lastPosition.get(); }
    virtual PositionError* lastError() const { return m_lastError.get(); }

    void positionChanged(PassRefPtr<Geoposition>);
    void errorOccurred(PassRefPtr<PositionError>);

private:
    GeolocationServiceMac(GeolocationServiceClient*);

    RetainPtr<CLLocationManager> m_locationManager;
    RetainPtr<WebCoreCoreLocationObserver> m_objcObserver;
    
    RefPtr<Geoposition> m_lastPosition;
    RefPtr<PositionError> m_lastError;
};
    
} // namespace WebCore

#endif // ENABLE(GEOLOCATION)

#endif // GeolocationServiceMac_h
