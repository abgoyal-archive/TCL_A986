

#ifndef GeolocationServiceMock_h
#define GeolocationServiceMock_h

#include "GeolocationService.h"
#include "Timer.h"
#include <wtf/HashSet.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

// This class provides a mock implementation of a GeolocationService for testing
// purposes. It allows the position or error that will be reported by this class
// to be set manually using the setPosition and setError methods. Objects of
// this class call back to their respective GeolocationServiceClient with the
// position or error every time either of these is updated.
class GeolocationServiceMock : public GeolocationService {
  public:
    static GeolocationService* create(GeolocationServiceClient*);

    GeolocationServiceMock(GeolocationServiceClient*);
    virtual ~GeolocationServiceMock();

#if PLATFORM(ANDROID)
    // TODO: Upstream to webkit.org. See https://bugs.webkit.org/show_bug.cgi?id=34082
    virtual bool startUpdating(PositionOptions*, bool suspend);
#else
    virtual bool startUpdating(PositionOptions*);
#endif
    virtual void stopUpdating();

    static void setPosition(PassRefPtr<Geoposition> position);
    static void setError(PassRefPtr<PositionError> position);

    virtual Geoposition* lastPosition() const { return s_lastPosition->get(); }
    virtual PositionError* lastError() const { return s_lastError->get(); }

  private:
    static void makeGeolocationCallbackFromAllInstances();
    void makeGeolocationCallback();

    void timerFired(Timer<GeolocationServiceMock>*);

    static void initStatics();
    static void cleanUpStatics();

    typedef HashSet<GeolocationServiceMock*> GeolocationServiceSet;
    static GeolocationServiceSet* s_instances;

    static RefPtr<Geoposition>* s_lastPosition;
    static RefPtr<PositionError>* s_lastError;

    Timer<GeolocationServiceMock> m_timer;

    bool m_isActive;
};

} // namespace WebCore

#endif // GeolocationServiceMock_h
