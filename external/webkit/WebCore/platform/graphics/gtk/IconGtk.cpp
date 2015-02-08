

#include "config.h"
#include "Icon.h"

#include "CString.h"
#include "GraphicsContext.h"
#include "MIMETypeRegistry.h"
#include "PassRefPtr.h"

#include <gtk/gtk.h>

namespace WebCore {

Icon::Icon()
    : m_icon(0)
{
}

Icon::~Icon()
{
    if (m_icon)
        g_object_unref(m_icon);
}

static String lookupIconName(String MIMEType)
{
    /*
     Lookup an appropriate icon according to either the Icon Naming Spec
     or conventional Gnome icon names respectively.

     See http://standards.freedesktop.org/icon-naming-spec/icon-naming-spec-latest.html

     The icon theme is probed for the following names:
     1. media-subtype
     2. gnome-mime-media-subtype
     3. media-x-generic
     4. gnome-mime-media

     In the worst case it falls back to the stock file icon.
    */
    int pos = MIMEType.find('/');
    if(pos >= 0) {
        String media = MIMEType.substring(0, pos);
        String subtype = MIMEType.substring(pos + 1);
        GtkIconTheme* iconTheme = gtk_icon_theme_get_default();
        String iconName = media + "-" + subtype;
        if(gtk_icon_theme_has_icon(iconTheme, iconName.utf8().data()))
            return iconName;
        iconName = "gnome-mime-" + media + "-" + subtype;
        if(gtk_icon_theme_has_icon(iconTheme, iconName.utf8().data()))
            return iconName;
        iconName = media + "-x-generic";
        if(gtk_icon_theme_has_icon(iconTheme, iconName.utf8().data()))
            return iconName;
        iconName = media + "gnome-mime-" + media;
        if(gtk_icon_theme_has_icon(iconTheme, iconName.utf8().data()))
            return iconName;
    }
    return GTK_STOCK_FILE;
}

PassRefPtr<Icon> Icon::createIconForFiles(const Vector<String>& filenames)
{
    if (filenames.isEmpty())
        return 0;

    if (filenames.size() == 1) {
        if (!g_path_skip_root(filenames[0].utf8().data()))
            return 0;

        String MIMEType = MIMETypeRegistry::getMIMETypeForPath(filenames[0]);
        String iconName = lookupIconName(MIMEType);

        RefPtr<Icon> icon = adoptRef(new Icon);
        icon->m_icon = gtk_icon_theme_load_icon(gtk_icon_theme_get_default(), iconName.utf8().data(), 16, GTK_ICON_LOOKUP_USE_BUILTIN, 0);
        if (!icon->m_icon)
            return 0;
        return icon.release();
    }

    //FIXME: Implement this
    return 0;
}

void Icon::paint(GraphicsContext* context, const IntRect& rect)
{
    if (context->paintingDisabled())
        return;

    // TODO: Scale/clip the image if necessary.
    cairo_t* cr = context->platformContext();
    cairo_save(cr);
    gdk_cairo_set_source_pixbuf(cr, m_icon, rect.x(), rect.y());
    cairo_paint(cr);
    cairo_restore(cr);
}

}
