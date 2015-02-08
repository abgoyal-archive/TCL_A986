

#ifndef V8Counters_h
#define V8Counters_h

#if USE(V8)

#ifdef ANDROID_INSTRUMENT

#include <PlatformString.h>
#include <v8.h>
#include <wtf/HashMap.h>

namespace WebCore {

class V8Counters {
public:
    // Counter callbacks, see v8.h
    static int* counterForName(const char* name);

    static void* createHistogram(const char* name,
                                 int min,
                                 int max,
                                 size_t buckets);

    static void addHistogramSample(void* histogram, int sample);

    static void initCounters();
    static void dumpCounters();
private:
    class Counter {
    public:
        Counter(bool isHistogram);

        int count() { return m_count; }
        int sampleTotal() { return m_sampleTotal; }
        bool isHistogram() { return m_isHistogram; }
        void addSample(int32_t sample);

        operator int*() { return &m_count; }
    private:
        int m_count;
        int m_sampleTotal;
        bool m_isHistogram;
    };

    static HashMap<String, Counter*> m_counters;
};

}

#endif // ANDROID_INSTRUMENT
#endif // USE(V8)
#endif // V8Counters_h
