

#ifndef GeolocationServiceBridge_h
#define GeolocationServiceBridge_h

#include "JNIUtility.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

class GeolocationServiceAndroid;
class Geoposition;

// GeolocationServiceBridge is the bridge to the Java implementation. It manages
// the lifetime of the Java object. It is an implementation detail of
// GeolocationServiceAndroid.
class GeolocationServiceBridge {
public:
    typedef GeolocationServiceAndroid ListenerInterface;
    GeolocationServiceBridge(ListenerInterface* listener);
    ~GeolocationServiceBridge();

    void start();
    void stop();
    void setEnableGps(bool enable);

    // Static wrapper functions to hide JNI nastiness.
    static void newLocationAvailable(JNIEnv *env, jclass, jlong nativeObject, jobject location);
    static void newErrorAvailable(JNIEnv *env, jclass, jlong nativeObject, jstring message);
    static PassRefPtr<Geoposition> toGeoposition(JNIEnv *env, const jobject &location);

private:
    void startJavaImplementation();
    void stopJavaImplementation();

    ListenerInterface* m_listener;
    jobject m_javaGeolocationServiceObject;
};

} // namespace WebCore

#endif // GeolocationServiceBridge_h
