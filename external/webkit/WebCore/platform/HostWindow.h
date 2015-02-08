

#ifndef HostWindow_h
#define HostWindow_h

#include <wtf/Noncopyable.h>
#include "Widget.h"

namespace WebCore {

class HostWindow : public Noncopyable {
public:
    virtual ~HostWindow() { }

    // The repaint method asks the host window to repaint a rect in the window's coordinate space.  The
    // contentChanged boolean indicates whether or not the Web page content actually changed (or if a repaint
    // of unchanged content is being requested).
    virtual void repaint(const IntRect&, bool contentChanged, bool immediate = false, bool repaintContentOnly = false) = 0;
    virtual void scroll(const IntSize& scrollDelta, const IntRect& rectToScroll, const IntRect& clipRect) = 0;

    // The paint method just causes a synchronous update of the window to happen for platforms that need it (Windows).
    void paint() { repaint(IntRect(), false, true); }
    
    // Methods for doing coordinate conversions to and from screen coordinates.
    virtual IntPoint screenToWindow(const IntPoint&) const = 0;
    virtual IntRect windowToScreen(const IntRect&) const = 0;

    // Method for retrieving the native client of the page.
    virtual PlatformPageClient platformPageClient() const = 0;
    
    // For scrolling a rect into view recursively.  Useful in the cases where a WebView is embedded inside some containing
    // platform-specific ScrollView.
    virtual void scrollRectIntoView(const IntRect&, const ScrollView*) const = 0;

    // To notify WebKit of scrollbar mode changes.
    virtual void scrollbarsModeDidChange() const = 0;
};

} // namespace WebCore

#endif // HostWindow_h
