

#ifndef GeolocationService_h
#define GeolocationService_h

#include <wtf/Noncopyable.h>

namespace WebCore {

class GeolocationService;
class Geoposition;
class PositionError;
class PositionOptions;

class GeolocationServiceClient {
public:
    virtual ~GeolocationServiceClient() { }
    virtual void geolocationServicePositionChanged(GeolocationService*) = 0;
    virtual void geolocationServiceErrorOccurred(GeolocationService*) = 0;
};

class GeolocationService : public Noncopyable {
public:
    static GeolocationService* create(GeolocationServiceClient*);
    virtual ~GeolocationService() { }
    
#if PLATFORM(ANDROID)
    // TODO: Upstream to webkit.org. See https://bugs.webkit.org/show_bug.cgi?id=34082
    virtual bool startUpdating(PositionOptions*, bool suspend) { return false; }
#else
    virtual bool startUpdating(PositionOptions*) { return false; }
#endif
    virtual void stopUpdating() { }
    
    virtual void suspend() { }
    virtual void resume() { }

    virtual Geoposition* lastPosition() const { return 0; }
    virtual PositionError* lastError() const { return 0; }

    void positionChanged();
    void errorOccurred();

    static void useMock();

protected:
    GeolocationService(GeolocationServiceClient*);

private:
    GeolocationServiceClient* m_geolocationServiceClient;

    typedef GeolocationService* (FactoryFunction)(GeolocationServiceClient*);
    static FactoryFunction* s_factoryFunction;
};

} // namespace WebCore

#endif // GeolocationService_h
