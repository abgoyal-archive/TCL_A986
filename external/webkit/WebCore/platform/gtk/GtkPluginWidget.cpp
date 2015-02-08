

#include "config.h"
#include "GtkPluginWidget.h"

#include "GraphicsContext.h"
#include "ScrollView.h"

#include <gtk/gtk.h>

namespace WebCore {

GtkPluginWidget::GtkPluginWidget(GtkWidget* widget)
    : Widget(widget)
{
    gtk_widget_hide(widget);
}

void GtkPluginWidget::invalidateRect(const IntRect& _rect)
{
    /* no need to */
    if (GTK_WIDGET_NO_WINDOW(platformWidget()))
        return;

    GdkWindow* window = platformWidget()->window;
    if (!window)
        return;

    GdkRectangle rect = _rect;
    gdk_window_invalidate_rect(window, &rect, FALSE);
}

void GtkPluginWidget::frameRectsChanged()
{
    IntRect rect = frameRect();
    IntPoint loc = parent()->contentsToWindow(rect.location());
    GtkAllocation allocation = { loc.x(), loc.y(), rect.width(), rect.height() };

    gtk_widget_set_size_request(platformWidget(), rect.width(), rect.height());
    gtk_widget_size_allocate(platformWidget(), &allocation);
    gtk_widget_show(platformWidget());
}

void GtkPluginWidget::paint(GraphicsContext* context, const IntRect& rect)
{
    if (!context->gdkExposeEvent())
        return;

    /* only paint widgets with NO_WINDOW this way */
    if (!GTK_WIDGET_NO_WINDOW(platformWidget()))
        return;

    GtkWidget* widget = platformWidget();
    ASSERT(GTK_WIDGET_NO_WINDOW(widget));

    GdkEvent* event = gdk_event_new(GDK_EXPOSE);
    event->expose = *context->gdkExposeEvent();
    event->expose.area = static_cast<GdkRectangle>(rect);

    IntPoint loc = parent()->contentsToWindow(rect.location());

    event->expose.area.x = loc.x();
    event->expose.area.y = loc.y();

    event->expose.region = gdk_region_rectangle(&event->expose.area);

    /*
     * This will be unref'ed by gdk_event_free.
     */
    g_object_ref(event->expose.window);

    /*
     * If we are going to paint do the translation and GtkAllocation manipulation.
     */
    if (!gdk_region_empty(event->expose.region))
        gtk_widget_send_expose(widget, event);

    gdk_event_free(event);
}

}
