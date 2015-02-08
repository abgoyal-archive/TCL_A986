

#ifndef RuntimeEnabledFeatures_h
#define RuntimeEnabledFeatures_h

namespace WebCore {

// A class that stores static enablers for all experimental features. Note that
// the method names must line up with the JavaScript method they enable for code
// generation to work properly.

class RuntimeEnabledFeatures {
public:
    static void setLocalStorageEnabled(bool isEnabled) { isLocalStorageEnabled = isEnabled; }
    static bool localStorageEnabled() { return isLocalStorageEnabled; }

    static void setSessionStorageEnabled(bool isEnabled) { isSessionStorageEnabled = isEnabled; }
    static bool sessionStorageEnabled() { return isSessionStorageEnabled; }

    static void setWebkitNotificationsEnabled(bool isEnabled) { isWebkitNotificationsEnabled = isEnabled; }
    static bool webkitNotificationsEnabled() { return isWebkitNotificationsEnabled; }

    static void setApplicationCacheEnabled(bool isEnabled) { isApplicationCacheEnabled = isEnabled; }
    static bool applicationCacheEnabled() { return isApplicationCacheEnabled; }

    static void setGeolocationEnabled(bool isEnabled) { isGeolocationEnabled = isEnabled; }
    static bool geolocationEnabled() { return isGeolocationEnabled; }

    static void setIndexedDBEnabled(bool isEnabled) { isIndexedDBEnabled = isEnabled; }
    static bool indexedDBEnabled() { return isIndexedDBEnabled; }

#if ENABLE(VIDEO)
    static bool audioEnabled();
    static bool htmlMediaElementEnabled();
    static bool htmlAudioElementEnabled();
    static bool htmlVideoElementEnabled();
    static bool mediaErrorEnabled();
#endif

#if ENABLE(SHARED_WORKERS)
    static bool sharedWorkerEnabled();
#endif

#if ENABLE(WEB_SOCKETS)
    static bool webSocketEnabled();
#endif

#if ENABLE(DATABASE)
    static bool openDatabaseEnabled();
#endif

private:
    // Never instantiate.
    RuntimeEnabledFeatures() { }

    static bool isLocalStorageEnabled;
    static bool isSessionStorageEnabled;
    static bool isWebkitNotificationsEnabled;
    static bool isApplicationCacheEnabled;
    static bool isGeolocationEnabled;
    static bool isIndexedDBEnabled;
};

} // namespace WebCore

#endif // RuntimeEnabledFeatures_h
