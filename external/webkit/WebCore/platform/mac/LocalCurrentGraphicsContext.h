

#include <wtf/Noncopyable.h>

#ifdef __OBJC__
@class NSGraphicsContext;
#else
class NSGraphicsContext;
#endif

namespace WebCore {

class GraphicsContext;
    
// This class automatically saves and restores the current NSGraphicsContext for
// functions which call out into AppKit and rely on the currentContext being set
class LocalCurrentGraphicsContext : public Noncopyable {
public:
    LocalCurrentGraphicsContext(GraphicsContext* graphicsContext);
    ~LocalCurrentGraphicsContext();

private:
    GraphicsContext* m_savedGraphicsContext;
    NSGraphicsContext* m_savedNSGraphicsContext;
};

}
