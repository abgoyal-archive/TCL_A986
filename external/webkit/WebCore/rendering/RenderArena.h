

#ifndef RenderArena_h
#define RenderArena_h

#include "Arena.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

static const size_t gMaxRecycledSize = 400;

class RenderArena : public Noncopyable {
public:
    RenderArena(unsigned arenaSize = 4096);
    ~RenderArena();

    // Memory management functions
    void* allocate(size_t);
    void free(size_t, void*);

#ifdef ANDROID_INSTRUMENT
    size_t reportPoolSize() const;
#endif

private:
    // Underlying arena pool
    ArenaPool m_pool;

    // The recycler array is sparse with the indices being multiples of 4,
    // i.e., 0, 4, 8, 12, 16, 20, ...
    void* m_recyclers[gMaxRecycledSize >> 2];
};

} // namespace WebCore

#endif // RenderArena_h
