

#include "config.h"
#include "WebRuntimeFeatures.h"

#include "Database.h"
#include "RuntimeEnabledFeatures.h"
#include "WebMediaPlayerClientImpl.h"
#include "WebSocket.h"

using namespace WebCore;

namespace WebKit {

void WebRuntimeFeatures::enableDatabase(bool enable)
{
#if ENABLE(DATABASE)
    Database::setIsAvailable(enable);
#endif
}

bool WebRuntimeFeatures::isDatabaseEnabled()
{
#if ENABLE(DATABASE)
    return Database::isAvailable();
#else
    return false;
#endif
}

void WebRuntimeFeatures::enableLocalStorage(bool enable)
{
#if ENABLE(DOM_STORAGE)
    RuntimeEnabledFeatures::setLocalStorageEnabled(enable);
#endif
}

bool WebRuntimeFeatures::isLocalStorageEnabled()
{
#if ENABLE(DOM_STORAGE)
    return RuntimeEnabledFeatures::localStorageEnabled();
#else
    return false;
#endif
}

void WebRuntimeFeatures::enableSessionStorage(bool enable)
{
#if ENABLE(DOM_STORAGE)
    RuntimeEnabledFeatures::setSessionStorageEnabled(enable);
#endif
}

bool WebRuntimeFeatures::isSessionStorageEnabled()
{
#if ENABLE(DOM_STORAGE)
    return RuntimeEnabledFeatures::sessionStorageEnabled();
#else
    return false;
#endif
}

void WebRuntimeFeatures::enableMediaPlayer(bool enable)
{
#if ENABLE(VIDEO)
    WebMediaPlayerClientImpl::setIsEnabled(enable);
#endif
}

bool WebRuntimeFeatures::isMediaPlayerEnabled()
{
#if ENABLE(VIDEO)
    return WebMediaPlayerClientImpl::isEnabled();
#else
    return false;
#endif
}

void WebRuntimeFeatures::enableSockets(bool enable)
{
#if ENABLE(WEB_SOCKETS)
    WebSocket::setIsAvailable(enable);
#endif
}

bool WebRuntimeFeatures::isSocketsEnabled()
{
#if ENABLE(WEB_SOCKETS)
    return WebSocket::isAvailable();
#else
    return false;
#endif
}

void WebRuntimeFeatures::enableNotifications(bool enable)
{
#if ENABLE(NOTIFICATIONS)
    RuntimeEnabledFeatures::setWebkitNotificationsEnabled(enable);
#endif
}

bool WebRuntimeFeatures::isNotificationsEnabled()
{
#if ENABLE(NOTIFICATIONS)
    return RuntimeEnabledFeatures::webkitNotificationsEnabled();
#else
    return false;
#endif
}

void WebRuntimeFeatures::enableApplicationCache(bool enable)
{
#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    RuntimeEnabledFeatures::setApplicationCacheEnabled(enable);
#endif
}

bool WebRuntimeFeatures::isApplicationCacheEnabled()
{
#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    return RuntimeEnabledFeatures::applicationCacheEnabled();
#else
    return false;
#endif
}

void WebRuntimeFeatures::enableGeolocation(bool enable)
{
#if ENABLE(GEOLOCATION)
    RuntimeEnabledFeatures::setGeolocationEnabled(enable);
#endif
}

bool WebRuntimeFeatures::isGeolocationEnabled()
{
#if ENABLE(GEOLOCATION)
    return RuntimeEnabledFeatures::geolocationEnabled();
#else
    return false;
#endif
}

void WebRuntimeFeatures::enableIndexedDatabase(bool enable)
{
#if ENABLE(INDEXED_DATABASE)
    RuntimeEnabledFeatures::setIndexedDBEnabled(enable);
#endif
}

bool WebRuntimeFeatures::isIndexedDatabaseEnabled()
{
#if ENABLE(INDEXED_DATABASE)
    return RuntimeEnabledFeatures::indexedDBEnabled();
#else
    return false;
#endif
}

} // namespace WebKit
