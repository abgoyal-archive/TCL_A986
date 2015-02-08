

#include "config.h"
#include "Logging.h"


namespace WebCore {

void InitializeLoggingChannelsIfNecessary()
{
    // FIXME: Should read the logging channels from a file.
    static bool haveInitializedLoggingChannels = false;
    if (haveInitializedLoggingChannels)
        return;

    haveInitializedLoggingChannels = true;

    LogEvents.state = WTFLogChannelOn;
    LogFrames.state = WTFLogChannelOn;
    LogLoading.state = WTFLogChannelOn;
    LogPlatformLeaks.state = WTFLogChannelOn;
}

} // namespace WebCore

