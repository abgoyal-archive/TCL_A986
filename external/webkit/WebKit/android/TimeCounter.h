

#ifndef TIME_COUNTER_H
#define TIME_COUNTER_H

#include "hardware_legacy/qemu_tracing.h"

namespace WebCore {

class KURL;

}

namespace android {

uint32_t getThreadMsec();

#ifdef ANDROID_INSTRUMENT

class TimeCounter {
public:
    enum Type {
        // function base counters
        CSSParseTimeCounter,
        JavaScriptTimeCounter,
        JavaScriptInitTimeCounter,
        JavaScriptParseTimeCounter,
        JavaScriptExecuteTimeCounter,
        CalculateStyleTimeCounter,
        JavaCallbackTimeCounter,
        ParsingTimeCounter,
        LayoutTimeCounter,
        // file base counters
        NativeCallbackTimeCounter,  // WebCoreFrameBridge.cpp
        ResourceTimeCounter,        // WebCoreResourceLoader.cpp
        SharedTimerTimeCounter,     // JavaBridge.cpp
        WebViewCoreBuildNavTimeCounter,
        WebViewCoreRecordTimeCounter,
        WebViewCoreTimeCounter,     // WebViewCore.cpp
        WebViewUIDrawTimeCounter,
        TotalTimeCounterCount
    };

    static void record(enum Type type, const char* functionName);
    static void recordNoCounter(enum Type type, const char* functionName);
    static void report(const WebCore::KURL& , int live, int dead, size_t arenaSize);
    static void reportNow();
    static void reset();
    static void start(enum Type type);
private:
    static uint32_t sStartWebCoreThreadTime;
    static uint32_t sEndWebCoreThreadTime;
    static bool sRecordWebCoreTime;
    static uint32_t sTotalTimeUsed[TotalTimeCounterCount];
    static uint32_t sLastTimeUsed[TotalTimeCounterCount];
    static uint32_t sCounter[TotalTimeCounterCount];
    static uint32_t sLastCounter[TotalTimeCounterCount];
    static uint32_t sStartTime[TotalTimeCounterCount];
    friend class TimeCounterAuto;
};

class TimeCounterAuto {
public:
    TimeCounterAuto(TimeCounter::Type type) : 
        m_type(type), m_startTime(getThreadMsec()) {}
    ~TimeCounterAuto() {
        uint32_t time = getThreadMsec();
        TimeCounter::sEndWebCoreThreadTime = time;
        TimeCounter::sTotalTimeUsed[m_type] += time - m_startTime;
        TimeCounter::sCounter[m_type]++;
    }
private:
    TimeCounter::Type m_type;
    uint32_t m_startTime;
};

class QemuTracerAuto {
public:
    QemuTracerAuto() {
        if (!reentry_count)
            qemu_start_tracing();
        reentry_count++;
    }

    ~QemuTracerAuto() {
        reentry_count--;
        if (!reentry_count)
            qemu_stop_tracing();
    }
private:
    static int reentry_count;
};
#endif  // ANDROID_INSTRUMENT

}

#endif
