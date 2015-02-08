

#ifndef AutodrainedPool_h
#define AutodrainedPool_h

#include <wtf/Noncopyable.h>

#ifdef __OBJC__
@class NSAutoreleasePool;
#else
class NSAutoreleasePool;
#endif

namespace WebCore {

class AutodrainedPool : public Noncopyable {
public:
    AutodrainedPool(int iterationLimit = 1);
    ~AutodrainedPool();
    
    void cycle();
    
private:
#if PLATFORM(MAC)
    int m_iterationLimit;
    int m_iterationCount;
    NSAutoreleasePool* m_pool;
#endif
};

#if !PLATFORM(MAC)
inline AutodrainedPool::AutodrainedPool(int) { }
inline AutodrainedPool::~AutodrainedPool() { }
inline void AutodrainedPool::cycle() { }
#endif

} // namespace WebCore

#endif


