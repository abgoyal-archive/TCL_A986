

#include "config.h"
#include "RenderArena.h"

#include <stdlib.h>
#include <string.h>
#include <wtf/Assertions.h>

#define ROUNDUP(x, y) ((((x)+((y)-1))/(y))*(y))

namespace WebCore {

#ifndef NDEBUG

const int signature = 0xDBA00AEA;
const int signatureDead = 0xDBA00AED;

typedef struct {
    RenderArena* arena;
    size_t size;
    int signature;
} RenderArenaDebugHeader;

#endif

RenderArena::RenderArena(unsigned arenaSize)
{
    // Initialize the arena pool
    INIT_ARENA_POOL(&m_pool, "RenderArena", arenaSize);

    // Zero out the recyclers array
    memset(m_recyclers, 0, sizeof(m_recyclers));
}

RenderArena::~RenderArena()
{
    FinishArenaPool(&m_pool);
}

void* RenderArena::allocate(size_t size)
{
#ifndef NDEBUG
    // Use standard malloc so that memory debugging tools work.
    ASSERT(this);
    void* block = ::malloc(sizeof(RenderArenaDebugHeader) + size);
    RenderArenaDebugHeader* header = static_cast<RenderArenaDebugHeader*>(block);
    header->arena = this;
    header->size = size;
    header->signature = signature;
    return header + 1;
#else
    void* result = 0;

    // Ensure we have correct alignment for pointers.  Important for Tru64
    size = ROUNDUP(size, sizeof(void*));

    // Check recyclers first
    if (size < gMaxRecycledSize) {
        const int index = size >> 2;

        result = m_recyclers[index];
        if (result) {
            // Need to move to the next object
            void* next = *((void**)result);
            m_recyclers[index] = next;
        }
    }

    if (!result) {
        // Allocate a new chunk from the arena
        ARENA_ALLOCATE(result, &m_pool, size);
    }

    return result;
#endif
}

void RenderArena::free(size_t size, void* ptr)
{
#ifndef NDEBUG
    // Use standard free so that memory debugging tools work.
    RenderArenaDebugHeader* header = static_cast<RenderArenaDebugHeader*>(ptr) - 1;
    ASSERT(header->signature == signature);
    ASSERT_UNUSED(size, header->size == size);
    ASSERT(header->arena == this);
    header->signature = signatureDead;
    ::free(header);
#else
    // Ensure we have correct alignment for pointers.  Important for Tru64
    size = ROUNDUP(size, sizeof(void*));

    // See if it's a size that we recycle
    if (size < gMaxRecycledSize) {
        const int index = size >> 2;
        void* currentTop = m_recyclers[index];
        m_recyclers[index] = ptr;
        *((void**)ptr) = currentTop;
    }
#endif
}

#ifdef ANDROID_INSTRUMENT
size_t RenderArena::reportPoolSize() const
{
    return ReportPoolSize(&m_pool);
}
#endif

} // namespace WebCore
