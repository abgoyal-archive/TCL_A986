

#define LOG_TAG "WebCore"

#include "config.h"
#include "TimeCounter.h"

#include "CString.h"
#include "Cache.h"
#include "KURL.h"
#include "Node.h"
#include "SystemTime.h"
#include "StyleBase.h"
#include <utils/Log.h>
#include <wtf/CurrentTime.h>

#include <sys/time.h>
#include <time.h>

#if USE(JSC)
#include "JSDOMWindow.h"
#include <runtime/JSGlobalObject.h>
#include <runtime/JSLock.h>
#endif

using namespace WebCore;
using namespace WTF;
using namespace JSC;

void webkit_logcat(const char *fmt, ...)
{
    va_list ap;
    char buf[256];
    //FILE *fp = 0;

    memset(buf, 0, sizeof(buf));

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    LOGD("%s", buf);

    //fp = fopen("/sdcard/webkit_logcat", "a");
    //fseek(fp, 0, SEEK_END);
    //fprintf(fp, "%s\n", buf);
    //fclose(fp);
}

void webkit_logcat_data(const char *data, int size)
{
#define OCTET_PER_LINE 16

    char line[OCTET_PER_LINE*3+1];  // xx xx xx xx... (each octet occupies 3 chars)
    int i,j;
    int toCopy;
    
    for (i=0; i<size;) {
        toCopy = ((size-i)>=OCTET_PER_LINE ? OCTET_PER_LINE : (size-i));
        memset(line, 0, sizeof(line));
        for (j=0; j<toCopy; j++)
            sprintf(line+strlen(line), "%02x ", (unsigned char)data[i+j]);

        LOGD("\t%s", line);

        i += toCopy;
    }
}


namespace android {

uint32_t getThreadMsec()
{
#if defined(HAVE_POSIX_CLOCKS)
    struct timespec tm;

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tm);
    return tm.tv_sec * 1000LL + tm.tv_nsec / 1000000;
#else
    struct timeval now;
    struct timezone zone;

    gettimeofday(&now, &zone);
    return now.tv_sec * 1000LL + now.tv_usec / 1000;
#endif
}

#ifdef ANDROID_INSTRUMENT

static double sStartTotalTime;
static uint32_t sStartThreadTime;
static double sLastTotalTime;
static uint32_t sLastThreadTime;

uint32_t TimeCounter::sStartWebCoreThreadTime;
uint32_t TimeCounter::sEndWebCoreThreadTime;
bool TimeCounter::sRecordWebCoreTime;
uint32_t TimeCounter::sTotalTimeUsed[TimeCounter::TotalTimeCounterCount];
uint32_t TimeCounter::sLastTimeUsed[TimeCounter::TotalTimeCounterCount];
uint32_t TimeCounter::sCounter[TimeCounter::TotalTimeCounterCount];
uint32_t TimeCounter::sLastCounter[TimeCounter::TotalTimeCounterCount];
uint32_t TimeCounter::sStartTime[TimeCounter::TotalTimeCounterCount];

int QemuTracerAuto::reentry_count = 0;

static const char* timeCounterNames[] = {
    "css parsing", 
    "javascript",
    "javascript init",
    "javascript parsing",
    "javascript execution",
    "calculate style", 
    "Java callback (frame bridge)",
    "parsing (may include calcStyle or Java callback)", 
    "layout", 
    "native 1 (frame bridge)",
    "native 2 (resource load)", 
    "native 3 (shared timer)", 
    "build nav (webview core)",
    "record content (webview core)",
    "native 4 (webview core)",
    "draw content (webview ui)",
};

void TimeCounter::record(enum Type type, const char* functionName)
{
    recordNoCounter(type, functionName);
    sCounter[type]++;
}

void TimeCounter::recordNoCounter(enum Type type, const char* functionName)
{
    uint32_t time = sEndWebCoreThreadTime = getThreadMsec();
    uint32_t elapsed = time - sStartTime[type];
    sTotalTimeUsed[type] += elapsed;
    if (elapsed > 1000)
        LOGW("***** %s() used %d ms\n", functionName, elapsed);
}

void TimeCounter::report(const KURL& url, int live, int dead, size_t arenaSize)
{
    String urlString = url;
    int totalTime = static_cast<int>((currentTime() - sStartTotalTime) * 1000);
    int threadTime = getThreadMsec() - sStartThreadTime;
    LOGD("*-* Total load time: %d ms, thread time: %d ms for %s\n",
        totalTime, threadTime, urlString.utf8().data());
    for (Type type = (Type) 0; type < TotalTimeCounterCount; type 
            = (Type) (type + 1)) {
        char scratch[256];
        int index = sprintf(scratch, "*-* Total %s time: %d ms", 
            timeCounterNames[type], sTotalTimeUsed[type]);
        if (sCounter[type] > 0)
            sprintf(&scratch[index], " called %d times", sCounter[type]);
        LOGD("%s", scratch);
    }
    LOGD("Current cache has %d bytes live and %d bytes dead", live, dead);
    LOGD("Current render arena takes %d bytes", arenaSize);
#if USE(JSC)
    JSLock lock(false);
    Heap::Statistics jsHeapStatistics = JSDOMWindow::commonJSGlobalData()->heap.statistics();
    LOGD("Current JavaScript heap size is %d and has %d bytes free",
            jsHeapStatistics.size, jsHeapStatistics.free);
#endif
    LOGD("Current CSS styles use %d bytes", StyleBase::reportStyleSize());
    LOGD("Current DOM nodes use %d bytes", WebCore::Node::reportDOMNodesSize());
}

void TimeCounter::reportNow()
{
    double current = currentTime();
    uint32_t currentThread = getThreadMsec();
    int elapsedTime = static_cast<int>((current - sLastTotalTime) * 1000);
    int elapsedThreadTime = currentThread - sLastThreadTime;
    LOGD("*-* Elapsed time: %d ms, ui thread time: %d ms, webcore thread time:"
        " %d ms\n", elapsedTime, elapsedThreadTime, sEndWebCoreThreadTime -
        sStartWebCoreThreadTime);
    for (Type type = (Type) 0; type < TotalTimeCounterCount; type 
            = (Type) (type + 1)) {
        if (sTotalTimeUsed[type] == sLastTimeUsed[type])
            continue;
        char scratch[256];
        int index = sprintf(scratch, "*-* Diff %s time: %d ms",
            timeCounterNames[type], sTotalTimeUsed[type] - sLastTimeUsed[type]);
        if (sCounter[type] > sLastCounter[type])
            sprintf(&scratch[index], " called %d times", sCounter[type]
                - sLastCounter[type]);
        LOGD("%s", scratch);
    }
    memcpy(sLastTimeUsed, sTotalTimeUsed, sizeof(sTotalTimeUsed));
    memcpy(sLastCounter, sCounter, sizeof(sCounter));
    sLastTotalTime = current;
    sLastThreadTime = currentThread;
    sRecordWebCoreTime = true;
}

void TimeCounter::reset() {
    bzero(sTotalTimeUsed, sizeof(sTotalTimeUsed));
    bzero(sCounter, sizeof(sCounter));
    LOGD("*-* Start browser instrument\n");
    sStartTotalTime = currentTime();
    sStartThreadTime = getThreadMsec();
}

void TimeCounter::start(enum Type type)
{
    uint32_t time = getThreadMsec();
    if (sRecordWebCoreTime) {
        sStartWebCoreThreadTime = time;
        sRecordWebCoreTime = false;
    }
    sStartTime[type] = time;
}

#endif  // ANDROID_INSTRUMENT

}
