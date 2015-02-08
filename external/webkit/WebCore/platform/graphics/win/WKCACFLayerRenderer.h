

#ifndef WKCACFLayerRenderer_h
#define WKCACFLayerRenderer_h

#if USE(ACCELERATED_COMPOSITING)

#include "COMPtr.h"
#include "Timer.h"
#include "WKCACFLayer.h"

#include <wtf/Noncopyable.h>

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/RetainPtr.h>

#include <CoreGraphics/CGGeometry.h>

interface IDirect3DDevice9;
typedef struct _CACFContext* CACFContextRef;
typedef struct _CARenderContext CARenderContext;
typedef struct _CARenderOGLContext CARenderOGLContext;

namespace WebCore {

// FIXME: Currently there is a WKCACFLayerRenderer for each WebView and each
// has its own CARenderOGLContext and Direct3DDevice9, which is inefficient.
// (https://bugs.webkit.org/show_bug.cgi?id=31855)
class WKCACFLayerRenderer : public Noncopyable {
public:
    static PassOwnPtr<WKCACFLayerRenderer> create();
    ~WKCACFLayerRenderer();

    static bool acceleratedCompositingAvailable();
    static void didFlushContext(CACFContextRef);

    void setScrollFrame(const IntRect&);
    void setRootContents(CGImageRef);
    void setRootChildLayer(WebCore::PlatformLayer* layer);
    void setNeedsDisplay();
    void setHostWindow(HWND window) { m_hostWindow = window; createRenderer(); }

    void createRenderer();
    void destroyRenderer();
    void resize();
    void renderSoon();

protected:
    WKCACFLayer* rootLayer() const { return m_rootLayer.get(); }

private:
    WKCACFLayerRenderer();

    void renderTimerFired(Timer<WKCACFLayerRenderer>*);

    CGRect bounds() const;

    void initD3DGeometry();
    void resetDevice();

    void render(const Vector<CGRect>& dirtyRects = Vector<CGRect>());
    void paint();

    bool m_triedToCreateD3DRenderer;
    COMPtr<IDirect3DDevice9> m_d3dDevice;
    RefPtr<WKCACFLayer> m_rootLayer;
    RefPtr<WKCACFLayer> m_viewLayer;
    RefPtr<WKCACFLayer> m_scrollLayer;
    RefPtr<WKCACFLayer> m_rootChildLayer;
    RetainPtr<CACFContextRef> m_context;
    CARenderContext* m_renderContext;
    CARenderOGLContext* m_renderer;
    HWND m_hostWindow;
    Timer<WKCACFLayerRenderer> m_renderTimer;
    IntRect m_scrollFrame;

#ifndef NDEBUG
    bool m_printTree;
#endif
};

}

#endif // USE(ACCELERATED_COMPOSITING)

#endif // WKCACFLayerRenderer_h
