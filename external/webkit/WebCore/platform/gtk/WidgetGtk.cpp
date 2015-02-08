

#include "config.h"
#include "Widget.h"

#include "Cursor.h"
#include "FrameView.h"
#include "GraphicsContext.h"
#include "HostWindow.h"
#include "IntRect.h"
#include "RenderObject.h"

#include <gdk/gdk.h>
#include <gtk/gtk.h>

namespace WebCore {

static GdkCursor* lastSetCursor;

Widget::Widget(PlatformWidget widget)
{
    init(widget);
}

Widget::~Widget()
{
    ASSERT(!parent());
    releasePlatformWidget();
}

void Widget::setFocus()
{
    gtk_widget_grab_focus(platformWidget() ? platformWidget() : GTK_WIDGET(root()->hostWindow()->platformPageClient()));
}

static GdkDrawable* gdkDrawable(PlatformWidget widget)
{
    return widget ? widget->window : 0;
}
    
void Widget::setCursor(const Cursor& cursor)
{
    GdkCursor* platformCursor = cursor.impl();

    // http://bugs.webkit.org/show_bug.cgi?id=16388
    // [GTK] Widget::setCursor() gets called frequently
    //
    // gdk_window_set_cursor() in certain GDK backends seems to be an
    // expensive operation, so avoid it if possible.

    if (platformCursor == lastSetCursor)
        return;

    gdk_window_set_cursor(gdkDrawable(platformWidget()) ? GDK_WINDOW(gdkDrawable(platformWidget())) : GTK_WIDGET(root()->hostWindow()->platformPageClient())->window, platformCursor);
    lastSetCursor = platformCursor;
}

void Widget::show()
{
    if (!platformWidget())
         return;
    gtk_widget_show(platformWidget());
}

void Widget::hide()
{
    if (!platformWidget())
         return;
    gtk_widget_hide(platformWidget());
}

void Widget::paint(GraphicsContext* context, const IntRect& rect)
{
}

void Widget::setIsSelected(bool isSelected)
{
    if (!platformWidget())
        return;

    // See if the platformWidget has a webkit-widget-is-selected property
    // and set it afterwards.
    GParamSpec* spec = g_object_class_find_property(G_OBJECT_GET_CLASS(platformWidget()),
                                                    "webkit-widget-is-selected");
    if (!spec)
        return;

    g_object_set(platformWidget(), "webkit-widget-is-selected", isSelected, NULL);
}

IntRect Widget::frameRect() const
{
    return m_frame;
}

void Widget::setFrameRect(const IntRect& rect)
{
    m_frame = rect;
}

void Widget::releasePlatformWidget()
{
    if (!platformWidget())
         return;
    g_object_unref(platformWidget());
}

void Widget::retainPlatformWidget()
{
    if (!platformWidget())
         return;
    g_object_ref_sink(platformWidget());
}

}
