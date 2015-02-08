

#include "config.h"
#include "InitializeThreading.h"

#include "Collector.h"
#include "dtoa.h"
#include "Identifier.h"
#include "JSGlobalObject.h"
#include "UString.h"
#include <wtf/DateMath.h>
#include <wtf/Threading.h>

using namespace WTF;

namespace JSC {

#if OS(DARWIN) && ENABLE(JSC_MULTIPLE_THREADS)
static pthread_once_t initializeThreadingKeyOnce = PTHREAD_ONCE_INIT;
#endif

static void initializeThreadingOnce()
{
    WTF::initializeThreading();
    initializeUString();
    JSGlobalData::storeVPtrs();
#if ENABLE(JSC_MULTIPLE_THREADS)
    s_dtoaP5Mutex = new Mutex;
    initializeDates();
#endif
}

void initializeThreading()
{
#if OS(DARWIN) && ENABLE(JSC_MULTIPLE_THREADS)
    pthread_once(&initializeThreadingKeyOnce, initializeThreadingOnce);
#else
    static bool initializedThreading = false;
    if (!initializedThreading) {
        initializeThreadingOnce();
        initializedThreading = true;
    }
#endif
}

} // namespace JSC
