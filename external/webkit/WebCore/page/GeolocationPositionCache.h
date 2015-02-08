

#ifndef GeolocationPositionCache_h
#define GeolocationPositionCache_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>


namespace WebCore {

class Geoposition;
class String;

class GeolocationPositionCache {
  public:
    GeolocationPositionCache();
    ~GeolocationPositionCache();

    void setCachedPosition(Geoposition*);
    Geoposition* cachedPosition();
    static void setDatabasePath(const String&);

  private:
    static PassRefPtr<Geoposition> readFromDB();
    static void writeToDB(const Geoposition*);

    static int s_instances;
    static RefPtr<Geoposition>* s_cachedPosition;
    static String* s_databaseFile;
};

} // namespace WebCore

#endif // GeolocationPositionCache_h
