
#define LOG_TAG "WebCore"

#include "config.h"
#include "ScrollView.h"

#include "FloatRect.h"
#include "FrameView.h"
#include "IntRect.h"
#include "SkRegion.h"
#include "WebCoreFrameBridge.h"
#include "WebCoreViewBridge.h"
#include "WebViewCore.h"


namespace WebCore {

IntRect ScrollView::platformVisibleContentRect(bool includeScrollbars) const
{
    IntRect rect = platformWidget()->getBounds();
    // This makes subframes draw correctly, since subframes cannot scroll.
    if (parent())
        return IntRect(0, 0, rect.width(), rect.height());
    return rect;
}

IntSize ScrollView::platformContentsSize() const
{
    return m_contentsSize;
}

void ScrollView::platformSetScrollPosition(const WebCore::IntPoint& pt)
{
    if (parent()) // don't attempt to scroll subframes; they're fully visible
        return;
    android::WebViewCore::getWebViewCore(this)->scrollTo(pt.x(), pt.y());
}

void ScrollView::platformSetScrollbarModes()
{
    if (parent()) // no scrollbar for the subframes
        return;
    android::WebViewCore::getWebViewCore(this)->setScrollbarModes(m_horizontalScrollbarMode, m_verticalScrollbarMode);
}

void ScrollView::platformScrollbarModes(ScrollbarMode& h, ScrollbarMode& v) const
{
    // m_horizontalScrollbarMode and m_verticalScrollbarMode are set in ScrollView::setScrollbarModes()
    h = m_horizontalScrollbarMode;
    v = m_verticalScrollbarMode;
}

bool ScrollView::platformProhibitsScrolling()
{
    if (!isFrameView())
        return false;
    FrameView* view = static_cast<FrameView*>(this);
    // We want to ignore requests to scroll that were not initiated by the user.  An
    // example of this is when text is inserted into a textfield/area, which results in
    // a scroll.  We ignore this because we now how to do this ourselves in the UI thread.
    // An example of it being initiated by the user is if the user clicks an anchor
    // element which simply scrolls the page.
    return !android::WebFrame::getWebFrame(view->frame())->userInitiatedClick();
}

void ScrollView::platformRepaintContentRectangle(const IntRect &rect, bool now)
{
    android::WebViewCore::getWebViewCore(this)->contentInvalidate(rect);
}

#ifdef ANDROID_CAPTURE_OFFSCREEN_PAINTS
//  Compute the offscreen parts of the drawn rectangle by subtracting
//  vis from rect. This can compute up to four rectangular slices.
void ScrollView::platformOffscreenContentRectangle(const IntRect& vis, const IntRect& rect)
{
    SkRegion rectRgn = SkRegion(rect);
    rectRgn.op(vis, SkRegion::kDifference_Op);
    SkRegion::Iterator iter(rectRgn);
    for (; !iter.done(); iter.next()) {
        const SkIRect& diff = iter.rect();
        android::WebViewCore::getWebViewCore(this)->offInvalidate(diff);
    }
}
#endif

} // namespace WebCore
