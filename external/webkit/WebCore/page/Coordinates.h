

#ifndef Coordinates_h
#define Coordinates_h

#include "Event.h"
#include "PlatformString.h"
#include <wtf/RefCounted.h>

namespace WebCore {

typedef int ExceptionCode;

class Coordinates : public RefCounted<Coordinates> {
public:
    static PassRefPtr<Coordinates> create(double latitude, double longitude, bool providesAltitude, double altitude, double accuracy, bool providesAltitudeAccuracy, double altitudeAccuracy, bool providesHeading, double heading, bool providesSpeed, double speed) { return adoptRef(new Coordinates(latitude, longitude, providesAltitude, altitude, accuracy, providesAltitudeAccuracy, altitudeAccuracy, providesHeading, heading, providesSpeed, speed)); }
    
    double latitude() const { return m_latitude; }
    double longitude() const { return m_longitude; }
    double altitude() const { return m_altitude; }
    double accuracy() const { return m_accuracy; }
    double altitudeAccuracy() const { return m_altitudeAccuracy; }
    double heading() const { return m_heading; }
    double speed() const { return m_speed; }

    bool canProvideAltitude() const { return m_canProvideAltitude; }
    bool canProvideAltitudeAccuracy() const { return m_canProvideAltitudeAccuracy; }
    bool canProvideHeading() const { return m_canProvideHeading; }
    bool canProvideSpeed() const { return m_canProvideSpeed; }
    
private:
    Coordinates(double latitude, double longitude, bool providesAltitude, double altitude, double accuracy, bool providesAltitudeAccuracy, double altitudeAccuracy, bool providesHeading, double heading, bool providesSpeed, double speed)
        : m_latitude(latitude)
        , m_longitude(longitude)
        , m_altitude(altitude)
        , m_accuracy(accuracy)
        , m_altitudeAccuracy(altitudeAccuracy)
        , m_heading(heading)
        , m_speed(speed)
        , m_canProvideAltitude(providesAltitude)
        , m_canProvideAltitudeAccuracy(providesAltitudeAccuracy)
        , m_canProvideHeading(providesHeading)
        , m_canProvideSpeed(providesSpeed)
    {
    }

    double m_latitude;
    double m_longitude;
    double m_altitude;
    double m_accuracy;
    double m_altitudeAccuracy;
    double m_heading;
    double m_speed;
    
    bool m_canProvideAltitude;
    bool m_canProvideAltitudeAccuracy;
    bool m_canProvideHeading;
    bool m_canProvideSpeed;
};
    
} // namespace WebCore

#endif // Coordinates_h
