

// The functions in this file are used to configure the mock GeolocationService
// for the LayoutTests.

#include "config.h"

#include "Coordinates.h"
#include "GeolocationServiceMock.h"
#include "Geoposition.h"
#include "JavaSharedClient.h"
#include "PositionError.h"
#include "WebCoreJni.h"
#include <JNIHelp.h>
#include <JNIUtility.h>
#include <wtf/CurrentTime.h>

using namespace WebCore;

namespace android {

static const char* javaMockGeolocationClass = "android/webkit/MockGeolocation";

static void setPosition(JNIEnv* env, jobject, double latitude, double longitude, double accuracy)
{
    RefPtr<Coordinates> coordinates = Coordinates::create(latitude,
                                                          longitude,
                                                          false, 0.0,  // altitude,
                                                          accuracy,
                                                          false, 0.0,  // altitudeAccuracy,
                                                          false, 0.0,  // heading
                                                          false, 0.0);  // speed
    RefPtr<Geoposition> position = Geoposition::create(coordinates.release(), WTF::currentTime());
    GeolocationServiceMock::setPosition(position.release());
}

static void setError(JNIEnv* env, jobject, int code, jstring message)
{
    PositionError::ErrorCode codeEnum = static_cast<PositionError::ErrorCode>(code);
    String messageString = to_string(env, message);
    RefPtr<PositionError> error = PositionError::create(codeEnum, messageString);
    GeolocationServiceMock::setError(error.release());
}

static JNINativeMethod gMockGeolocationMethods[] = {
    { "nativeSetPosition", "(DDD)V", (void*) setPosition },
    { "nativeSetError", "(ILjava/lang/String;)V", (void*) setError }
};

int register_mock_geolocation(JNIEnv* env)
{
    jclass mockGeolocation = env->FindClass(javaMockGeolocationClass);
    LOG_ASSERT(mockGeolocation, "Unable to find class");
    return jniRegisterNativeMethods(env, javaMockGeolocationClass, gMockGeolocationMethods, NELEM(gMockGeolocationMethods));
}

}
