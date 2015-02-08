

#ifndef WebPopupMenuImpl_h
#define WebPopupMenuImpl_h

// FIXME: Add this to FramelessScrollViewClient.h
namespace WebCore { class FramelessScrollView; }

#include "FramelessScrollViewClient.h"
// FIXME: remove the relative paths once glue/ consumers are removed.
#include "../public/WebPoint.h"
#include "../public/WebPopupMenu.h"
#include "../public/WebSize.h"
#include <wtf/RefCounted.h>

namespace WebCore {
class Frame;
class FramelessScrollView;
class KeyboardEvent;
class Page;
class PlatformKeyboardEvent;
class Range;
class Widget;
}

namespace WebKit {
class WebKeyboardEvent;
class WebMouseEvent;
class WebMouseWheelEvent;
struct WebRect;

class WebPopupMenuImpl : public WebPopupMenu,
                         public WebCore::FramelessScrollViewClient,
                         public RefCounted<WebPopupMenuImpl> {
public:
    // WebWidget
    virtual void close();
    virtual WebSize size() { return m_size; }
    virtual void resize(const WebSize&);
    virtual void layout();
    virtual void paint(WebCanvas* canvas, const WebRect& rect);
    virtual bool handleInputEvent(const WebInputEvent&);
    virtual void mouseCaptureLost();
    virtual void setFocus(bool enable);
    virtual bool handleCompositionEvent(
        WebCompositionCommand command, int cursorPosition,
        int targetStart, int targetEnd, const WebString& text);
    virtual bool queryCompositionStatus(bool* enabled, WebRect* caretRect);
    virtual void setTextDirection(WebTextDirection direction);

    // WebPopupMenuImpl
    void Init(WebCore::FramelessScrollView* widget,
              const WebRect& bounds);

    WebWidgetClient* client() { return m_client; }

    void MouseMove(const WebMouseEvent&);
    void MouseLeave(const WebMouseEvent&);
    void MouseDown(const WebMouseEvent&);
    void MouseUp(const WebMouseEvent&);
    void MouseDoubleClick(const WebMouseEvent&);
    void MouseWheel(const WebMouseWheelEvent&);
    bool KeyEvent(const WebKeyboardEvent&);

   protected:
    friend class WebPopupMenu;  // For WebPopupMenu::create
    friend class WTF::RefCounted<WebPopupMenuImpl>;

    WebPopupMenuImpl(WebWidgetClient* client);
    ~WebPopupMenuImpl();

    // WebCore::HostWindow methods:
    virtual void repaint(
        const WebCore::IntRect&, bool contentChanged, bool immediate = false,
        bool repaintContentOnly = false);
    virtual void scroll(
        const WebCore::IntSize& scrollDelta, const WebCore::IntRect& scrollRect,
        const WebCore::IntRect& clipRect);
    virtual WebCore::IntPoint screenToWindow(const WebCore::IntPoint&) const;
    virtual WebCore::IntRect windowToScreen(const WebCore::IntRect&) const;
    virtual PlatformPageClient platformPageClient() const { return 0; }
    virtual void scrollRectIntoView(const WebCore::IntRect&, const WebCore::ScrollView*) const;
    virtual void scrollbarsModeDidChange() const;

    // WebCore::FramelessScrollViewClient methods:
    virtual void popupClosed(WebCore::FramelessScrollView*);

    WebWidgetClient* m_client;
    WebSize m_size;

    WebPoint m_lastMousePosition;

    // This is a non-owning ref.  The popup will notify us via popupClosed()
    // before it is destroyed.
    WebCore::FramelessScrollView* m_widget;
};

} // namespace WebKit

#endif
