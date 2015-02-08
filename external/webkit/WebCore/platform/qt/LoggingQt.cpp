

#include "config.h"
#include "Logging.h"
#include "PlatformString.h"

#include <QDebug>
#include <QStringList>

namespace WebCore {

void InitializeLoggingChannelsIfNecessary()
{
    static bool haveInitializedLoggingChannels = false;
    if (haveInitializedLoggingChannels)
        return;

    haveInitializedLoggingChannels = true;

    QByteArray loggingEnv = qgetenv("QT_WEBKIT_LOG");
    if (loggingEnv.isEmpty())
        return;

#if defined(NDEBUG)
    qWarning("This is a release build. Setting QT_WEBKIT_LOG will have no effect.");
#else
    QStringList channels = QString::fromLocal8Bit(loggingEnv).split(QLatin1String(","));
    for (int i = 0; i < channels.count(); i++) {
        if (WTFLogChannel* channel = getChannelFromName(channels.at(i)))
            channel->state = WTFLogChannelOn;
    }

    // By default we log calls to notImplemented(). This can be turned
    // off by setting the environment variable DISABLE_NI_WARNING to 1
    LogNotYetImplemented.state = WTFLogChannelOn;
#endif
}

} // namespace WebCore
