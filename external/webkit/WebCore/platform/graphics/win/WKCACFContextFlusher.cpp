

#include "config.h"

#if USE(ACCELERATED_COMPOSITING)

#include "WKCACFContextFlusher.h"

#include <wtf/StdLibExtras.h>
#include <QuartzCore/CACFContext.h>

namespace WebCore {

WKCACFContextFlusher& WKCACFContextFlusher::shared()
{
    DEFINE_STATIC_LOCAL(WKCACFContextFlusher, flusher, ());
    return flusher;
}

WKCACFContextFlusher::WKCACFContextFlusher()
{
}

WKCACFContextFlusher::~WKCACFContextFlusher()
{
}

void WKCACFContextFlusher::addContext(CACFContextRef context)
{
    ASSERT(context);

    m_contexts.add(context);
    CFRetain(context);
}

void WKCACFContextFlusher::removeContext(CACFContextRef context)
{
    ASSERT(context);

    ContextSet::iterator found = m_contexts.find(context);
    if (found == m_contexts.end())
        return;

    CFRelease(*found);
    m_contexts.remove(found);
}

void WKCACFContextFlusher::flushAllContexts()
{
    // addContext might get called beneath CACFContextFlush, and we don't want m_contexts to change while
    // we're iterating over it, so we move the contexts into a local ContextSet and iterate over that instead.
    ContextSet contextsToFlush;
    contextsToFlush.swap(m_contexts);

    ContextSet::const_iterator end = contextsToFlush.end();
    for (ContextSet::const_iterator it = contextsToFlush.begin(); it != end; ++it) {
        CACFContextRef context = *it;
        CACFContextFlush(context);
        CFRelease(context);
    }
}

}

#endif // USE(ACCELERATED_COMPOSITING)
