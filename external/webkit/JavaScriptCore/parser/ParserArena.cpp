

#include "config.h"
#include "ParserArena.h"

#include "Nodes.h"

namespace JSC {

ParserArena::ParserArena()
    : m_freeableMemory(0)
    , m_freeablePoolEnd(0)
    , m_identifierArena(new IdentifierArena)
{
}

inline void* ParserArena::freeablePool()
{
    ASSERT(m_freeablePoolEnd);
    return m_freeablePoolEnd - freeablePoolSize;
}

inline void ParserArena::deallocateObjects()
{
    if (m_freeablePoolEnd)
        fastFree(freeablePool());

    size_t size = m_freeablePools.size();
    for (size_t i = 0; i < size; ++i)
        fastFree(m_freeablePools[i]);

    size = m_deletableObjects.size();
    for (size_t i = 0; i < size; ++i) {
        ParserArenaDeletable* object = m_deletableObjects[i];
        object->~ParserArenaDeletable();
        fastFree(object);
    }
}

ParserArena::~ParserArena()
{
    deallocateObjects();
}

bool ParserArena::contains(ParserArenaRefCounted* object) const
{
    return m_refCountedObjects.find(object) != notFound;
}

ParserArenaRefCounted* ParserArena::last() const
{
    return m_refCountedObjects.last().get();
}

void ParserArena::removeLast()
{
    m_refCountedObjects.removeLast();
}

void ParserArena::reset()
{
    // Since this code path is used only when parsing fails, it's not bothering to reuse
    // any of the memory the arena allocated. We could improve that later if we want to
    // efficiently reuse the same arena.

    deallocateObjects();

    m_freeableMemory = 0;
    m_freeablePoolEnd = 0;
    m_identifierArena->clear();
    m_freeablePools.clear();
    m_deletableObjects.clear();
    m_refCountedObjects.clear();
}

void ParserArena::allocateFreeablePool()
{
    if (m_freeablePoolEnd)
        m_freeablePools.append(freeablePool());

    char* pool = static_cast<char*>(fastMalloc(freeablePoolSize));
    m_freeableMemory = pool;
    m_freeablePoolEnd = pool + freeablePoolSize;
    ASSERT(freeablePool() == pool);
}

bool ParserArena::isEmpty() const
{
    return !m_freeablePoolEnd
        && m_identifierArena->isEmpty()
        && m_freeablePools.isEmpty()
        && m_deletableObjects.isEmpty()
        && m_refCountedObjects.isEmpty();
}

void ParserArena::derefWithArena(PassRefPtr<ParserArenaRefCounted> object)
{
    m_refCountedObjects.append(object);
}

}
