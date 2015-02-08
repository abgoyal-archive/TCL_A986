

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

    // By default we log calls to notImplemented().
    LogNotYetImplemented.state = WTFLogChannelOn;
}

} // namespace WebCore

