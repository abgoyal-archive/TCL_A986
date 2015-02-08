

#include "config.h"
#include "CollectionCache.h"

namespace WebCore {

CollectionCache::CollectionCache()
    : version(0)
{
    reset();
}

inline void CollectionCache::copyCacheMap(NodeCacheMap& dest, const NodeCacheMap& src)
{
    ASSERT(dest.isEmpty());
    NodeCacheMap::const_iterator end = src.end();
    for (NodeCacheMap::const_iterator it = src.begin(); it != end; ++it)
        dest.add(it->first, new Vector<Element*>(*it->second));
}

CollectionCache::CollectionCache(const CollectionCache& other)
    : version(other.version)
    , current(other.current)
    , position(other.position)
    , length(other.length)
    , elementsArrayPosition(other.elementsArrayPosition)
    , hasLength(other.hasLength)
    , hasNameCache(other.hasNameCache)
{
    copyCacheMap(idCache, other.idCache);
    copyCacheMap(nameCache, other.nameCache);
}

void CollectionCache::swap(CollectionCache& other)
{
    std::swap(version, other.version);
    std::swap(current, other.current);
    std::swap(position, other.position);
    std::swap(length, other.length);
    std::swap(elementsArrayPosition, other.elementsArrayPosition);

    idCache.swap(other.idCache);
    nameCache.swap(other.nameCache);
    
    std::swap(hasLength, other.hasLength);
    std::swap(hasNameCache, other.hasNameCache);
}

CollectionCache::~CollectionCache()
{
    deleteAllValues(idCache);
    deleteAllValues(nameCache);
}

void CollectionCache::reset()
{
    current = 0;
    position = 0;
    length = 0;
    hasLength = false;
    elementsArrayPosition = 0;
    deleteAllValues(idCache);
    idCache.clear();
    deleteAllValues(nameCache);
    nameCache.clear();
    hasNameCache = false;
}

#if !ASSERT_DISABLED
void CollectionCache::checkConsistency()
{
    idCache.checkConsistency();
    nameCache.checkConsistency();
}
#endif

} // namespace WebCore
