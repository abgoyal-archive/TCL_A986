

#ifndef GeolocationServiceGtk_h
#define GeolocationServiceGtk_h

#include "GeolocationService.h"
#include "Geoposition.h"
#include "PositionError.h"
#include "RefPtr.h"

#include <geoclue/geoclue-master.h>
#include <geoclue/geoclue-position.h>

namespace WebCore {
    class GeolocationServiceGtk : public GeolocationService {
    public:
        static GeolocationService* create(GeolocationServiceClient*);
        ~GeolocationServiceGtk();

        virtual bool startUpdating(PositionOptions*);
        virtual void stopUpdating();

        virtual void suspend();
        virtual void resume();

        Geoposition* lastPosition() const;
        PositionError* lastError() const;

    private:
        GeolocationServiceGtk(GeolocationServiceClient*);

        void updateLocationInformation();
        void setError(PositionError::ErrorCode, const char* message);
        void updatePosition();

        static void position_changed(GeocluePosition*, GeocluePositionFields, int, double, double, double, GeoclueAccuracy*, GeolocationServiceGtk*);

    private:
        RefPtr<Geoposition> m_lastPosition;
        RefPtr<PositionError> m_lastError;

        // state objects
        GeoclueMasterClient* m_geoclueClient;
        GeocluePosition* m_geocluePosition;

        // Error and Position state
        double m_latitude;
        double m_longitude;
        double m_altitude;
        double m_accuracy;
        double m_altitudeAccuracy;
        int m_timestamp;
    };
}

#endif
