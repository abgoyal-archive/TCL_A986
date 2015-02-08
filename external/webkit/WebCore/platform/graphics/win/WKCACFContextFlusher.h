

#ifndef WKCACFContextFlusher_h
#define WKCACFContextFlusher_h

#if USE(ACCELERATED_COMPOSITING)

#include <wtf/Noncopyable.h>

#include <wtf/HashSet.h>

typedef struct _CACFContext* CACFContextRef;

namespace WebCore {

class WKCACFContextFlusher : public Noncopyable {
public:
    static WKCACFContextFlusher& shared();

    void addContext(CACFContextRef);
    void removeContext(CACFContextRef);

    void flushAllContexts();

private:
    WKCACFContextFlusher();
    ~WKCACFContextFlusher();

    typedef HashSet<CACFContextRef> ContextSet;
    ContextSet m_contexts;
};

}

#endif // USE(ACCELERATED_COMPOSITING)

#endif // WKCACFContextFlusher_h
