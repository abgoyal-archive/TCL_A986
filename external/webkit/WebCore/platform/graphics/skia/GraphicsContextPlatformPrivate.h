

#ifndef GraphicsContextPlatformPrivate_h
#define GraphicsContextPlatformPrivate_h

#include <wtf/Noncopyable.h>

class PlatformContextSkia;

namespace WebCore {

// This class just holds onto a PlatformContextSkia for GraphicsContext.
class GraphicsContextPlatformPrivate : public Noncopyable {
public:
    GraphicsContextPlatformPrivate(PlatformContextSkia* platformContext)
        : m_context(platformContext) { }

    PlatformContextSkia* context() { return m_context; }

private:
    // Non-owning pointer to the PlatformContext.
    PlatformContextSkia* m_context;
};

}  // namespace WebCore

#endif  // GraphicsContextPlatformPrivate_h
