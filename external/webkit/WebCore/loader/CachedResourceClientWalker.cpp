

#include "config.h"
#include "CachedResourceClientWalker.h"

namespace WebCore {

CachedResourceClientWalker::CachedResourceClientWalker(const HashCountedSet<CachedResourceClient*>& set)
    : m_clientSet(set), m_clientVector(set.size()), m_index(0)
{
    typedef HashCountedSet<CachedResourceClient*>::const_iterator Iterator;
    Iterator end = set.end();
    size_t clientIndex = 0;
    for (Iterator current = set.begin(); current != end; ++current)
        m_clientVector[clientIndex++] = current->first;
}

CachedResourceClient* CachedResourceClientWalker::next()
{
    size_t size = m_clientVector.size();
    while (m_index < size) {
        CachedResourceClient* next = m_clientVector[m_index++];
        if (m_clientSet.contains(next))
            return next;
    }

    return 0;
}

}
