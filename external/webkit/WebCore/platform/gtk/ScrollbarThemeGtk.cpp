

#include "config.h"
#include "ScrollbarThemeGtk.h"

#include "gtkdrawing.h"
#include <gtk/gtk.h>

namespace WebCore {

ScrollbarTheme* ScrollbarTheme::nativeTheme()
{
    static ScrollbarThemeGtk theme;
    return &theme;
}

ScrollbarThemeGtk::~ScrollbarThemeGtk()
{
}

int ScrollbarThemeGtk::scrollbarThickness(ScrollbarControlSize controlSize)
{
    static int size;
    if (!size) {
        MozGtkScrollbarMetrics metrics;
        moz_gtk_get_scrollbar_metrics(&metrics);
        size = metrics.slider_width;
    }
    return size;
}

}

