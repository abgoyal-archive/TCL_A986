

#include "config.h"
#include "RuntimeEnabledFeatures.h"

#include "Database.h"
#include "MediaPlayer.h"
#include "SharedWorkerRepository.h"
#include "WebSocket.h"

namespace WebCore {

bool RuntimeEnabledFeatures::isLocalStorageEnabled = true;
bool RuntimeEnabledFeatures::isSessionStorageEnabled = true;
bool RuntimeEnabledFeatures::isWebkitNotificationsEnabled = false;
bool RuntimeEnabledFeatures::isApplicationCacheEnabled = true;
bool RuntimeEnabledFeatures::isGeolocationEnabled = true;
bool RuntimeEnabledFeatures::isIndexedDBEnabled = false;

#if ENABLE(VIDEO)

bool RuntimeEnabledFeatures::audioEnabled()
{
    return MediaPlayer::isAvailable();
}

bool RuntimeEnabledFeatures::htmlMediaElementEnabled()
{
    return MediaPlayer::isAvailable();
}

bool RuntimeEnabledFeatures::htmlAudioElementEnabled()
{
    return MediaPlayer::isAvailable();
}

bool RuntimeEnabledFeatures::htmlVideoElementEnabled()
{
    return MediaPlayer::isAvailable();
}

bool RuntimeEnabledFeatures::mediaErrorEnabled()
{
    return MediaPlayer::isAvailable();
}

#endif

#if ENABLE(SHARED_WORKERS)
bool RuntimeEnabledFeatures::sharedWorkerEnabled()
{
    return SharedWorkerRepository::isAvailable();
}
#endif

#if ENABLE(WEB_SOCKETS)
bool RuntimeEnabledFeatures::webSocketEnabled()
{
    return WebSocket::isAvailable();
}
#endif

#if ENABLE(DATABASE)
bool RuntimeEnabledFeatures::openDatabaseEnabled()
{
    return Database::isAvailable();
}
#endif

} // namespace WebCore
