

#include "config.h"
#include "Logging.h"
#include "PlatformString.h"

#include <glib.h>
#include <string.h>

namespace WebCore {

// Inspired by the code used by the Qt port

void InitializeLoggingChannelsIfNecessary()
{
    static bool didInitializeLoggingChannels = false;
    if (didInitializeLoggingChannels)
        return;

    didInitializeLoggingChannels = true;

    char* logEnv = getenv("WEBKIT_DEBUG");
    if (!logEnv)
        return;

    // we set up the logs anyway because some of our logging, such as
    // soup's is available in release builds
#if defined(NDEBUG)
    g_warning("WEBKIT_DEBUG is not empty, but this is a release build. Notice that many log messages will only appear in a debug build.");
#endif

    char** logv = g_strsplit(logEnv, " ", -1);

    for (int i = 0; logv[i]; i++) {
        if (WTFLogChannel* channel = getChannelFromName(logv[i]))
            channel->state = WTFLogChannelOn;
    }

    g_strfreev(logv);

    // to disable logging notImplemented set the DISABLE_NI_WARNING
    // environment variable to 1
    LogNotYetImplemented.state = WTFLogChannelOn;
}

} // namespace WebCore
