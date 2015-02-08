

#include "config.h"
#include "Logging.h"
#include "PlatformString.h"

namespace WebCore {

WTFLogChannel LogNotYetImplemented = { 0x00000001, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogFrames =            { 0x00000010, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogLoading =           { 0x00000020, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogPopupBlocking =     { 0x00000040, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogEvents =            { 0x00000080, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogEditing =           { 0x00000100, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogTextConversion =    { 0x00000200, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogIconDatabase =      { 0x00000400, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogSQLDatabase =       { 0x00000800, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogSpellingAndGrammar ={ 0x00001000, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogBackForward =       { 0x00002000, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogHistory =           { 0x00004000, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogPageCache =         { 0x00008000, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogPlatformLeaks =     { 0x00010000, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogNetwork =           { 0x00100000, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogFTP =               { 0x00200000, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogThreading =         { 0x00400000, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogStorageAPI =        { 0x00800000, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogMedia =             { 0x01000000, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel LogPlugins =           { 0x02000000, "WebCoreLogLevel", WTFLogChannelOff };
WTFLogChannel LogArchives =          { 0x04000000, "WebCoreLogLevel", WTFLogChannelOff };

WTFLogChannel* getChannelFromName(const String& channelName)
{
    if (!(channelName.length() >= 2))
        return 0;

    if (equalIgnoringCase(channelName, String("BackForward")))
        return &LogBackForward;

    if (equalIgnoringCase(channelName, String("Editing")))
        return &LogEditing;

    if (equalIgnoringCase(channelName, String("Events")))
        return &LogEvents;

    if (equalIgnoringCase(channelName, String("Frames")))
        return &LogFrames;

    if (equalIgnoringCase(channelName, String("FTP")))
        return &LogFTP;

    if (equalIgnoringCase(channelName, String("History")))
        return &LogHistory;

    if (equalIgnoringCase(channelName, String("IconDatabase")))
        return &LogIconDatabase;

    if (equalIgnoringCase(channelName, String("Loading")))
        return &LogLoading;

    if (equalIgnoringCase(channelName, String("Media")))
        return &LogMedia;

    if (equalIgnoringCase(channelName, String("Network")))
        return &LogNetwork;

    if (equalIgnoringCase(channelName, String("NotYetImplemented")))
        return &LogNotYetImplemented;

    if (equalIgnoringCase(channelName, String("PageCache")))
        return &LogPageCache;

    if (equalIgnoringCase(channelName, String("PlatformLeaks")))
        return &LogPlatformLeaks;

    if (equalIgnoringCase(channelName, String("Plugins")))
        return &LogPlugins;

    if (equalIgnoringCase(channelName, String("PopupBlocking")))
        return &LogPopupBlocking;

    if (equalIgnoringCase(channelName, String("SpellingAndGrammar")))
        return &LogSpellingAndGrammar;

    if (equalIgnoringCase(channelName, String("SQLDatabase")))
        return &LogSQLDatabase;

    if (equalIgnoringCase(channelName, String("StorageAPI")))
        return &LogStorageAPI;

    if (equalIgnoringCase(channelName, String("TextConversion")))
        return &LogTextConversion;

    if (equalIgnoringCase(channelName, String("Threading")))
        return &LogThreading;

    return 0;
}

}
