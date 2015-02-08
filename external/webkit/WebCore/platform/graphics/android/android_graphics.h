

#ifndef android_graphics_DEFINED
#define android_graphics_DEFINED

#include "DrawExtra.h"
#include "IntRect.h"
#include "SkTypes.h"
#include "wtf/Vector.h"

namespace WebCore {
    class GraphicsContext;
}

SkCanvas* android_gc2canvas(GraphicsContext* gc);

namespace android {

class CachedFrame;
class CachedNode;
class CachedRoot;
class WebViewCore;

// Data and methods for cursor rings

// used to inflate node cache entry
#define CURSOR_RING_HIT_TEST_RADIUS 5

// used to inval rectangle enclosing pressed state of ring
#define CURSOR_RING_OUTER_DIAMETER SkFixedToScalar(SkIntToFixed(13)>>2) // 13/4 == 3.25

class CursorRing : public DrawExtra {
public:
    enum Flavor {
        NORMAL_FLAVOR,
        FAKE_FLAVOR,
        NORMAL_ANIMATING,
        FAKE_ANIMATING,
        ANIMATING_COUNT = 2
    };

    CursorRing(WebViewCore* core) : m_viewImpl(core) {}
    virtual ~CursorRing() {}
    virtual void draw(SkCanvas* , LayerAndroid* );
    bool setup();
private:
    friend class WebView;
    WebViewCore* m_viewImpl; // copy for convenience
    WTF::Vector<IntRect> m_rings;
    IntRect m_bounds;
    const CachedRoot* m_root;
    const CachedFrame* m_frame;
    const CachedNode* m_node;
    Flavor m_flavor;
    bool m_followedLink;
    bool m_isButton;
};

}

#endif
