

#include "config.h"
#include "DNS.h"

#include "StringHash.h"
#include "Timer.h"
#include <wtf/HashSet.h>
#include <wtf/RetainPtr.h>
#include <wtf/StdLibExtras.h>

#if PLATFORM(WIN)
#include "LoaderRunLoopCF.h"
#endif

#ifdef BUILDING_ON_TIGER
// This function is available on Tiger, but not declared in the CFRunLoop.h header on Tiger.
extern "C" CFRunLoopRef CFRunLoopGetMain();
#endif

namespace WebCore {

// When resolve queue is empty, we fire async resolution requests immediately (which is important if the prefetch is triggered by hovering).
// But during page parsing, we should coalesce identical requests to avoid stressing out CFHost.
const int namesToResolveImmediately = 4;

// Coalesce prefetch requests for this long before sending them out.
const double coalesceDelay = 1.0;

// For a page has links to many outside sites, it is likely that the system DNS resolver won't be able to cache them all anyway, and we don't want
// to negatively affect other appications' performance, by pushing their cached entries out, too.
// If we end up with lots of names to prefetch, some will be dropped.
const int maxRequestsToSend = 64;

class DNSResolveQueue : public TimerBase {
public:
    static DNSResolveQueue& shared();
    void add(const String&);
    void decrementRequestCount();

private:
    DNSResolveQueue();

    void resolve(const String&);
    virtual void fired();
    HashSet<String> m_names;
    int m_requestsInFlight;
};

DNSResolveQueue::DNSResolveQueue()
    : m_requestsInFlight(0)
{
}

DNSResolveQueue& DNSResolveQueue::shared()
{
    DEFINE_STATIC_LOCAL(DNSResolveQueue, names, ());
    return names;
}

void DNSResolveQueue::add(const String& name)
{
    // If there are no names queued, and few enough are in flight, resolve immediately (the mouse may be over a link).
    if (!m_names.size()) {
        if (atomicIncrement(&m_requestsInFlight) <= namesToResolveImmediately) {
            resolve(name);
            return;
        }
        atomicDecrement(&m_requestsInFlight);
    }
    m_names.add(name);
    if (!isActive())
        startOneShot(coalesceDelay);
}

void DNSResolveQueue::decrementRequestCount()
{
    atomicDecrement(&m_requestsInFlight);
}

void DNSResolveQueue::fired()
{
    int requestsAllowed = maxRequestsToSend - m_requestsInFlight;

    for (HashSet<String>::iterator iter = m_names.begin(); iter != m_names.end() && requestsAllowed > 0; ++iter, --requestsAllowed) {
        atomicIncrement(&m_requestsInFlight);
        resolve(*iter);
    }

    // It's better to skip some names than to clog the queue.
    m_names.clear();
}

static void clientCallback(CFHostRef theHost, CFHostInfoType, const CFStreamError*, void*)
{
    DNSResolveQueue::shared().decrementRequestCount(); // It's ok to call shared() from a secondary thread, the static variable has already been initialized by now.
    CFRelease(theHost);
}

void DNSResolveQueue::resolve(const String& hostname)
{
    ASSERT(isMainThread());

    RetainPtr<CFStringRef> hostnameCF(AdoptCF, hostname.createCFString());
    RetainPtr<CFHostRef> host(AdoptCF, CFHostCreateWithName(0, hostnameCF.get()));
    if (!host) {
        atomicDecrement(&m_requestsInFlight);
        return;
    }
    CFHostClientContext context = { 0, 0, 0, 0, 0 };
    Boolean result = CFHostSetClient(host.get(), clientCallback, &context);
    ASSERT_UNUSED(result, result);
#if !PLATFORM(WIN)
    CFHostScheduleWithRunLoop(host.get(), CFRunLoopGetMain(), kCFRunLoopCommonModes);
#else
    // On Windows, we run a separate thread with CFRunLoop, which is where clientCallback will be called.
    CFHostScheduleWithRunLoop(host.get(), loaderRunLoop(), kCFRunLoopDefaultMode);
#endif
    CFHostStartInfoResolution(host.get(), kCFHostAddresses, 0);
    host.releaseRef(); // The host will be released from clientCallback().
}

void prefetchDNS(const String& hostname)
{
    ASSERT(isMainThread());
    if (hostname.isEmpty())
        return;
    DNSResolveQueue::shared().add(hostname);
}

}
