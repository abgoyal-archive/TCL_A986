

#ifndef GeolocationControllerClient_h
#define GeolocationControllerClient_h

namespace WebCore {

class GeolocationPosition;

class GeolocationControllerClient {
public:
    virtual void geolocationDestroyed() = 0;

    virtual void startUpdating() = 0;
    virtual void stopUpdating() = 0;
    virtual GeolocationPosition* lastPosition() = 0;

protected:
    virtual ~GeolocationControllerClient() { }
};

} // namespace WebCore

#endif // GeolocationControllerClient_h
