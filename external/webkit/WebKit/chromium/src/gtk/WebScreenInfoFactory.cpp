

#include "config.h"
#include "WebScreenInfoFactory.h"

#include "WebScreenInfo.h"
#include <gtk/gtk.h>

namespace WebKit {

WebScreenInfo WebScreenInfoFactory::screenInfo(GtkWidget* widget)
{
    WebScreenInfo results;
    results.depth = 32;
    results.depthPerComponent = 8;
    results.isMonochrome = false;

    if (!widget)
        return results;

    GdkScreen* screen = gtk_widget_get_screen(widget);

    results.rect = WebRect(
        0, 0, gdk_screen_get_width(screen), gdk_screen_get_height(screen));

    // I don't know of a way to query the "maximize" size of the window (e.g.
    // screen size less sidebars etc) since this is something which only the
    // window manager knows.
    results.availableRect = results.rect;

    return results;
}

} // namespace WebKit
