
#ifndef WebKitLogging_h
#define WebKitLogging_h

#include <wtf/Assertions.h>

#ifndef LOG_CHANNEL_PREFIX
#define LOG_CHANNEL_PREFIX WebKitLog
#endif


extern WTFLogChannel WebKitLogTextInput;
extern WTFLogChannel WebKitLogTiming;
extern WTFLogChannel WebKitLogLoading;
extern WTFLogChannel WebKitLogFontCache;
extern WTFLogChannel WebKitLogFontSubstitution;
extern WTFLogChannel WebKitLogFontSelection;
extern WTFLogChannel WebKitLogDownload;
extern WTFLogChannel WebKitLogDocumentLoad;
extern WTFLogChannel WebKitLogPlugins;
extern WTFLogChannel WebKitLogEvents;
extern WTFLogChannel WebKitLogView;
extern WTFLogChannel WebKitLogRedirect;
extern WTFLogChannel WebKitLogPageCache;
extern WTFLogChannel WebKitLogCacheSizes;
extern WTFLogChannel WebKitLogFormDelegate;
extern WTFLogChannel WebKitLogFileDatabaseActivity;
extern WTFLogChannel WebKitLogHistory;
extern WTFLogChannel WebKitLogBindings;
extern WTFLogChannel WebKitLogEncoding;
extern WTFLogChannel WebKitLogLiveConnect;
extern WTFLogChannel WebKitLogBackForward;
extern WTFLogChannel WebKitLogProgress;
extern WTFLogChannel WebKitLogPluginEvents;
extern WTFLogChannel WebKitLogIconDatabase;

void WebKitInitializeLoggingChannelsIfNecessary(void);

#endif
