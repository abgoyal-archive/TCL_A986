

#include "config.h"
#include "FramelessScrollView.h"

#include "FramelessScrollViewClient.h"

namespace WebCore {

FramelessScrollView::~FramelessScrollView()
{
    // Remove native scrollbars now before we lose the connection to the HostWindow.
    setHasHorizontalScrollbar(false);
    setHasVerticalScrollbar(false);
}

void FramelessScrollView::invalidateScrollbarRect(Scrollbar* scrollbar, const IntRect& rect)
{
    // Add in our offset within the ScrollView.
    IntRect dirtyRect = rect;
    dirtyRect.move(scrollbar->x(), scrollbar->y());
    invalidateRect(dirtyRect);
}

bool FramelessScrollView::isActive() const
{
    // FIXME
    return true;
}

void FramelessScrollView::invalidateRect(const IntRect& rect)
{
    if (HostWindow* h = hostWindow())
        h->repaint(rect, true);
}

HostWindow* FramelessScrollView::hostWindow() const
{
    return const_cast<FramelessScrollViewClient*>(m_client);
}

IntRect FramelessScrollView::windowClipRect(bool clipToContents) const
{
    return contentsToWindow(visibleContentRect(!clipToContents));
}

void FramelessScrollView::paintContents(GraphicsContext*, const IntRect& damageRect)
{
}

void FramelessScrollView::contentsResized()
{
}

void FramelessScrollView::visibleContentsResized()
{
}

} // namespace WebCore
