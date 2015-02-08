

#include "config.h"
#include "GRefPtrGtk.h"

#include <glib.h>
#include <gtk/gtk.h>

namespace WTF {

template <> GtkTargetList* refGPtr(GtkTargetList* ptr)
{
    if (ptr)
        gtk_target_list_ref(ptr);
    return ptr;
}

template <> void derefGPtr(GtkTargetList* ptr)
{
    if (ptr)
        gtk_target_list_unref(ptr);
}

template <> GdkCursor* refGPtr(GdkCursor* ptr)
{
    if (ptr)
        gdk_cursor_ref(ptr);
    return ptr;
}

template <> void derefGPtr(GdkCursor* ptr)
{
    if (ptr)
        gdk_cursor_unref(ptr);
}

}
