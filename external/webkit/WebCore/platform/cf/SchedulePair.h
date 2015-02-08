

#ifndef SchedulePair_h
#define SchedulePair_h

#include "PlatformString.h"
#include <wtf/HashSet.h>
#include <wtf/RetainPtr.h>

#if PLATFORM(MAC)
#ifdef __OBJC__
@class NSRunLoop;
#else
class NSRunLoop;
#endif
#endif

namespace WebCore {

class SchedulePair : public RefCounted<SchedulePair> {
public:
    static PassRefPtr<SchedulePair> create(CFRunLoopRef runLoop, CFStringRef mode) { return adoptRef(new SchedulePair(runLoop, mode)); }

#if PLATFORM(MAC)
    static PassRefPtr<SchedulePair> create(NSRunLoop* runLoop, CFStringRef mode) { return adoptRef(new SchedulePair(runLoop, mode)); }
    NSRunLoop* nsRunLoop() const { return m_nsRunLoop.get(); }
#endif

    CFRunLoopRef runLoop() const { return m_runLoop.get(); }
    CFStringRef mode() const { return m_mode.get(); }

    bool operator==(const SchedulePair& other) const;

private:
    SchedulePair(CFRunLoopRef, CFStringRef);

#if PLATFORM(MAC)
    SchedulePair(NSRunLoop*, CFStringRef);
    RetainPtr<NSRunLoop*> m_nsRunLoop;
#endif

    RetainPtr<CFRunLoopRef> m_runLoop;
    RetainPtr<CFStringRef> m_mode;
};

struct SchedulePairHash {
    static unsigned hash(const RefPtr<SchedulePair>& pair)
    {
        uintptr_t hashCodes[2] = { reinterpret_cast<uintptr_t>(pair->runLoop()), pair->mode() ? CFHash(pair->mode()) : 0 };
        return StringImpl::computeHash(reinterpret_cast<UChar*>(hashCodes), sizeof(hashCodes) / sizeof(UChar));
    }

    static bool equal(const RefPtr<SchedulePair>& a, const RefPtr<SchedulePair>& b) { return a == b; }

    static const bool safeToCompareToEmptyOrDeleted = true;
};

typedef HashSet<RefPtr<SchedulePair>, SchedulePairHash> SchedulePairHashSet;

} // namespace WebCore

#endif
