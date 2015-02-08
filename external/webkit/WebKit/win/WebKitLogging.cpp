
#include "config.h"
#include "WebKitLogging.h"

WTFLogChannel WebKitLogTextInput =              { 0x00000010, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogTiming =                 { 0x00000020, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogLoading =                { 0x00000040, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogFontCache =              { 0x00000100, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogFontSubstitution =       { 0x00000200, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogDownload =               { 0x00000800, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogDocumentLoad =           { 0x00001000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogPlugins =                { 0x00002000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogEvents =                 { 0x00010000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogView =                   { 0x00020000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogRedirect =               { 0x00040000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogPageCache =              { 0x00080000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogCacheSizes =             { 0x00100000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogFormDelegate =           { 0x00200000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogFileDatabaseActivity =   { 0x00400000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogHistory =                { 0x00800000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogBindings =               { 0x01000000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogFontSelection =          { 0x02000000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogEncoding =               { 0x04000000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogLiveConnect =            { 0x08000000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogBackForward =            { 0x10000000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogProgress =               { 0x20000000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogPluginEvents =           { 0x40000000, "WebKitLogLevel", WTFLogChannelOff };
WTFLogChannel WebKitLogIconDatabase =           { 0x80000000, "WebKitLogLevel", WTFLogChannelOff };

static void initializeLogChannel(WTFLogChannel *)
{
    // FIXME: Get the LogChannel preferences from somewhere, otherwise people will have to hard code
    // the logging channels they want to WTFLogChannelOn
}

void WebKitInitializeLoggingChannelsIfNecessary()
{
    static bool haveInitializedLoggingChannels = false;
    if (haveInitializedLoggingChannels)
        return;
    haveInitializedLoggingChannels = true;
    
    initializeLogChannel(&WebKitLogTextInput);
    initializeLogChannel(&WebKitLogTiming);
    initializeLogChannel(&WebKitLogLoading);
    initializeLogChannel(&WebKitLogFontCache);
    initializeLogChannel(&WebKitLogFontSubstitution);
    initializeLogChannel(&WebKitLogDownload);
    initializeLogChannel(&WebKitLogDocumentLoad);
    initializeLogChannel(&WebKitLogPlugins);
    initializeLogChannel(&WebKitLogEvents);
    initializeLogChannel(&WebKitLogView);
    initializeLogChannel(&WebKitLogRedirect);
    initializeLogChannel(&WebKitLogPageCache);
    initializeLogChannel(&WebKitLogCacheSizes);
    initializeLogChannel(&WebKitLogFormDelegate);
    initializeLogChannel(&WebKitLogFileDatabaseActivity);
    initializeLogChannel(&WebKitLogHistory);
    initializeLogChannel(&WebKitLogBindings);
    initializeLogChannel(&WebKitLogFontSelection);
    initializeLogChannel(&WebKitLogEncoding);
    initializeLogChannel(&WebKitLogLiveConnect);
    initializeLogChannel(&WebKitLogBackForward);
    initializeLogChannel(&WebKitLogProgress);
    initializeLogChannel(&WebKitLogPluginEvents);
    initializeLogChannel(&WebKitLogIconDatabase);
}

