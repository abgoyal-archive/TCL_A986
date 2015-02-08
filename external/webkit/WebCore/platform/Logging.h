

#ifndef Logging_h
#define Logging_h

#include <wtf/Assertions.h>

#ifndef LOG_CHANNEL_PREFIX
#define LOG_CHANNEL_PREFIX Log
#endif

namespace WebCore {

    class String;

    extern WTFLogChannel LogNotYetImplemented;
    extern WTFLogChannel LogFrames;
    extern WTFLogChannel LogLoading;
    extern WTFLogChannel LogPopupBlocking;
    extern WTFLogChannel LogEvents;
    extern WTFLogChannel LogEditing;
    extern WTFLogChannel LogTextConversion;
    extern WTFLogChannel LogIconDatabase;
    extern WTFLogChannel LogSQLDatabase;
    extern WTFLogChannel LogSpellingAndGrammar;
    extern WTFLogChannel LogBackForward;
    extern WTFLogChannel LogHistory;
    extern WTFLogChannel LogPageCache;
    extern WTFLogChannel LogPlatformLeaks;
    extern WTFLogChannel LogNetwork;
    extern WTFLogChannel LogFTP;
    extern WTFLogChannel LogThreading;
    extern WTFLogChannel LogStorageAPI;
    extern WTFLogChannel LogMedia;
    extern WTFLogChannel LogPlugins;
    extern WTFLogChannel LogArchives;

    void InitializeLoggingChannelsIfNecessary();
    WTFLogChannel* getChannelFromName(const String& channelName);
}

#endif // Logging_h
