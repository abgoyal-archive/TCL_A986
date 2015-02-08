

#if USE(V8)

#ifdef ANDROID_INSTRUMENT

#define LOG_TAG "WebCore"

#include "config.h"
#include "V8Counters.h"
#include "NotImplemented.h"

#include <CString.h>
#include <StringHash.h>
#include <utils/Log.h>

namespace WebCore {

V8Counters::Counter::Counter(bool isHistogram)
    : m_count(0), m_sampleTotal(0), m_isHistogram(isHistogram) { }

void V8Counters::Counter::addSample(int sample)
{
  m_count++;
  m_sampleTotal += sample;
}

HashMap<String, V8Counters::Counter*> V8Counters::m_counters;

// static
int* V8Counters::counterForName(const char* name)
{
    Counter* counter = m_counters.get(name);
    if (!counter) {
        counter = new Counter(false);
        m_counters.add(name, counter);
    }
    return *counter;
}

// static
void* V8Counters::createHistogram(const char* name, int min, int max,
        size_t buckets)
{
    Counter* counter = new Counter(true);
    m_counters.add(name, counter);
    return counter;
}

// static
void V8Counters::addHistogramSample(void* histogram, int sample)
{
    Counter* counter = reinterpret_cast<Counter*>(histogram);
    counter->addSample(sample);
}

// static
void V8Counters::initCounters()
{
    static bool isInitialized = false;
    if (!isInitialized) {
        v8::V8::SetCounterFunction(counterForName);
        v8::V8::SetCreateHistogramFunction(createHistogram);
        v8::V8::SetAddHistogramSampleFunction(addHistogramSample);
        isInitialized = true;
    }
}

// static
void V8Counters::dumpCounters()
{
    LOGD("+----------------------------------------+-------------+\n");
    LOGD("| Name                                   | Value       |\n");
    LOGD("+----------------------------------------+-------------+\n");
    typedef HashMap<String, V8Counters::Counter*>::iterator CounterIterator;
    for (CounterIterator iter = m_counters.begin(); iter != m_counters.end(); ++iter) {
        Counter* counter = iter->second;
        if (counter->isHistogram()) {
            LOGD("| c:%-36s | %11i |\n", iter->first.latin1().data(), counter->count());
            LOGD("| t:%-36s | %11i |\n", iter->first.latin1().data(), counter->sampleTotal());
          } else {
            LOGD("| %-38s | %11i |\n", iter->first.latin1().data(), counter->count());
          }
    }
    LOGD("+----------------------------------------+-------------+\n");
}

}

#endif // ANDROID_INSTRUMENT

#endif // USE(V8)
