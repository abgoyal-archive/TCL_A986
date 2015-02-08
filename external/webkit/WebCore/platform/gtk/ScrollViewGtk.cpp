

#include "config.h"
#include "ScrollView.h"

#include "ChromeClient.h"
#include "FloatRect.h"
#include "Frame.h"
#include "FrameView.h"
#include "GraphicsContext.h"
#include "HostWindow.h"
#include "IntRect.h"
#include "Page.h"
#include "PlatformMouseEvent.h"
#include "PlatformWheelEvent.h"
#include "ScrollbarGtk.h"
#include "ScrollbarTheme.h"

#include <gtk/gtk.h>

using namespace std;

namespace WebCore {

void ScrollView::platformInit()
{
    m_horizontalAdjustment = 0;
    m_verticalAdjustment = 0;
}

void ScrollView::platformDestroy()
{
    m_horizontalAdjustment = 0;
    m_verticalAdjustment = 0;
}

PassRefPtr<Scrollbar> ScrollView::createScrollbar(ScrollbarOrientation orientation)
{
    if (orientation == HorizontalScrollbar && m_horizontalAdjustment)
        return ScrollbarGtk::createScrollbar(this, orientation, m_horizontalAdjustment);
    else if (orientation == VerticalScrollbar && m_verticalAdjustment)
        return ScrollbarGtk::createScrollbar(this, orientation, m_verticalAdjustment);
    else
        return Scrollbar::createNativeScrollbar(this, orientation, RegularScrollbar);
}

void ScrollView::setGtkAdjustments(GtkAdjustment* hadj, GtkAdjustment* vadj, bool resetValues)
{
    ASSERT(!hadj == !vadj);

    m_horizontalAdjustment = hadj;
    m_verticalAdjustment = vadj;

    // Reset the adjustments to a sane default
    if (m_horizontalAdjustment) {
        ScrollbarGtk* hScrollbar = reinterpret_cast<ScrollbarGtk*>(horizontalScrollbar());
        if (hScrollbar)
            hScrollbar->attachAdjustment(m_horizontalAdjustment);

        ScrollbarGtk* vScrollbar = reinterpret_cast<ScrollbarGtk*>(verticalScrollbar());
        if (vScrollbar)
            vScrollbar->attachAdjustment(m_verticalAdjustment);

        // We used to reset everything to 0 here, but when page cache
        // is enabled we reuse FrameViews that are cached. Since their
        // size is not going to change when being restored, (which is
        // what would cause the upper limit in the adjusments to be
        // set in the normal case), we make sure they are up-to-date
        // here. This is needed for the parent scrolling widget to be
        // able to report correct values.
        m_horizontalAdjustment->lower = 0;
        m_horizontalAdjustment->upper = resetValues ? 0 : frameRect().width();
        m_horizontalAdjustment->value = resetValues ? 0 : scrollOffset().width();
        gtk_adjustment_changed(m_horizontalAdjustment);
        gtk_adjustment_value_changed(m_horizontalAdjustment);

        m_verticalAdjustment->lower = 0;
        m_verticalAdjustment->upper = resetValues ? 0 : frameRect().height();
        m_verticalAdjustment->value = resetValues ? 0 : scrollOffset().height();
        gtk_adjustment_changed(m_verticalAdjustment);
        gtk_adjustment_value_changed(m_verticalAdjustment);
    } else {
        ScrollbarGtk* hScrollbar = reinterpret_cast<ScrollbarGtk*>(horizontalScrollbar());
        if (hScrollbar)
            hScrollbar->detachAdjustment();

        ScrollbarGtk* vScrollbar = reinterpret_cast<ScrollbarGtk*>(verticalScrollbar());
        if (vScrollbar)
            vScrollbar->detachAdjustment();
    }

    /* reconsider having a scrollbar */
    setHasVerticalScrollbar(false);
    setHasHorizontalScrollbar(false);
}

void ScrollView::platformAddChild(Widget* child)
{
    if (!GTK_IS_SOCKET(child->platformWidget()))
        gtk_container_add(GTK_CONTAINER(hostWindow()->platformPageClient()), child->platformWidget());
}

void ScrollView::platformRemoveChild(Widget* child)
{
    GtkWidget* parent;

    // HostWindow can be NULL here. If that's the case
    // let's grab the child's parent instead.
    if (hostWindow())
        parent = GTK_WIDGET(hostWindow()->platformPageClient());
    else
        parent = GTK_WIDGET(child->platformWidget()->parent);

    if (GTK_IS_CONTAINER(parent) && parent == child->platformWidget()->parent)
        gtk_container_remove(GTK_CONTAINER(parent), child->platformWidget());
}

IntRect ScrollView::visibleContentRect(bool includeScrollbars) const
{
    if (!m_horizontalAdjustment)
        return IntRect(IntPoint(m_scrollOffset.width(), m_scrollOffset.height()),
                       IntSize(max(0, width() - (verticalScrollbar() && !includeScrollbars ? verticalScrollbar()->width() : 0)),
                               max(0, height() - (horizontalScrollbar() && !includeScrollbars ? horizontalScrollbar()->height() : 0))));

    // Main frame.
    GtkWidget* measuredWidget = hostWindow()->platformPageClient();
    GtkWidget* parent = gtk_widget_get_parent(measuredWidget);

    // We may not be in a widget that displays scrollbars, but we may
    // have other kinds of decoration that make us smaller.
    if (parent && includeScrollbars)
        measuredWidget = parent;

    return IntRect(IntPoint(m_scrollOffset.width(), m_scrollOffset.height()),
                   IntSize(measuredWidget->allocation.width,
                           measuredWidget->allocation.height));
}

void ScrollView::setScrollbarModes(ScrollbarMode horizontalMode, ScrollbarMode verticalMode)
{
    if (horizontalMode == m_horizontalScrollbarMode && verticalMode == m_verticalScrollbarMode)
        return;

    m_horizontalScrollbarMode = horizontalMode;
    m_verticalScrollbarMode = verticalMode;

    // We don't really care about reporting policy changes on frames
    // that have no adjustments attached to them.
    if (!m_horizontalAdjustment) {
        updateScrollbars(scrollOffset());
        return;
    }

    if (!isFrameView())
        return;

    // For frames that do have adjustments attached, we want to report
    // policy changes, so that they may be applied to the widget to
    // which the WebView has been added, for instance.
    if (hostWindow())
        hostWindow()->scrollbarsModeDidChange();
}

}
