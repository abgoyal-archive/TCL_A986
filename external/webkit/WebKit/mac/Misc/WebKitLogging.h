

#import <wtf/Assertions.h>

#ifndef LOG_CHANNEL_PREFIX
#define LOG_CHANNEL_PREFIX WebKitLog
#endif

#ifdef __cplusplus
extern "C" {
#endif

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
extern WTFLogChannel WebKitLogTextInput;

void WebKitInitializeLoggingChannelsIfNecessary(void);

// FIXME: Why is this in the "logging" header file?
// Use WebCoreThreadViolationCheck instead for checks that throw an exception even in production builds.
#if !defined(NDEBUG) && !defined(DISABLE_THREAD_CHECK)
#define ASSERT_MAIN_THREAD() do \
    if (!pthread_main_np()) { \
        WTFReportAssertionFailure(__FILE__, __LINE__, WTF_PRETTY_FUNCTION, "<not running on main thread>"); \
        CRASH(); \
    } \
while (0)
#else
#define ASSERT_MAIN_THREAD() ((void)0)
#endif

void ReportDiscardedDelegateException(SEL delegateSelector, id exception);

#ifdef __cplusplus
}
#endif
