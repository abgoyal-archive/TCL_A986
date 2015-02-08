

#ifndef WebRuntimeFeatures_h
#define WebRuntimeFeatures_h

#include "WebCommon.h"

namespace WebKit {

// This class is used to enable runtime features of WebKit.  It is unspecified
// whether a feature is enabled by default.  In the future, a feature may be
// promoted from disabled by default to enabled by default once it reaches a
// certain level of maturity.
class WebRuntimeFeatures {
public:
    WEBKIT_API static void enableDatabase(bool);
    WEBKIT_API static bool isDatabaseEnabled();

    WEBKIT_API static void enableLocalStorage(bool);
    WEBKIT_API static bool isLocalStorageEnabled();

    WEBKIT_API static void enableSessionStorage(bool);
    WEBKIT_API static bool isSessionStorageEnabled();

    WEBKIT_API static void enableMediaPlayer(bool);
    WEBKIT_API static bool isMediaPlayerEnabled();

    WEBKIT_API static void enableSockets(bool);
    WEBKIT_API static bool isSocketsEnabled();

    WEBKIT_API static void enableNotifications(bool);
    WEBKIT_API static bool isNotificationsEnabled();

    WEBKIT_API static void enableApplicationCache(bool);
    WEBKIT_API static bool isApplicationCacheEnabled();

    WEBKIT_API static void enableGeolocation(bool);
    WEBKIT_API static bool isGeolocationEnabled();

    WEBKIT_API static void enableIndexedDatabase(bool);
    WEBKIT_API static bool isIndexedDatabaseEnabled();

private:
    WebRuntimeFeatures();
};

} // namespace WebKit

#endif
