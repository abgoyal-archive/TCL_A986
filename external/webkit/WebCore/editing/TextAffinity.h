

#ifndef TextAffinity_h
#define TextAffinity_h

#include <wtf/Platform.h>

#ifdef __OBJC__
#include <AppKit/NSTextView.h>
#endif

namespace WebCore {

// These match the AppKit values for these concepts.
// From NSTextView.h:
// NSSelectionAffinityUpstream = 0
// NSSelectionAffinityDownstream = 1
enum EAffinity { UPSTREAM = 0, DOWNSTREAM = 1 };

} // namespace WebCore

#ifdef __OBJC__

inline NSSelectionAffinity kit(WebCore::EAffinity affinity)
{
    return static_cast<NSSelectionAffinity>(affinity);
}

inline WebCore::EAffinity core(NSSelectionAffinity affinity)
{
    return static_cast<WebCore::EAffinity>(affinity);
}

#endif

#endif // TextAffinity_h
