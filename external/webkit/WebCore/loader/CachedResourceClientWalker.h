

#ifndef CachedResourceClientWalker_h
#define CachedResourceClientWalker_h

#include <wtf/HashCountedSet.h>
#include <wtf/Vector.h>

namespace WebCore {

    class CachedResourceClient;

    // Call this "walker" instead of iterator so people won't expect Qt or STL-style iterator interface.
    // Just keep calling next() on this. It's safe from deletions of items.
    class CachedResourceClientWalker {
    public:
        CachedResourceClientWalker(const HashCountedSet<CachedResourceClient*>&);
        CachedResourceClient* next();
    private:
        const HashCountedSet<CachedResourceClient*>& m_clientSet;
        Vector<CachedResourceClient*> m_clientVector;
        size_t m_index;
    };

}

#endif
