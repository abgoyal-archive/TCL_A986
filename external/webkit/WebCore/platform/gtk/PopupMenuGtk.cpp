

#include "config.h"
#include "PopupMenu.h"

#include "CString.h"
#include "FrameView.h"
#include "HostWindow.h"
#include "PlatformString.h"
#include <gtk/gtk.h>

namespace WebCore {

PopupMenu::PopupMenu(PopupMenuClient* client)
    : m_popupClient(client)
{
}

PopupMenu::~PopupMenu()
{
    if (m_popup) {
        g_signal_handlers_disconnect_matched(m_popup.get(), G_SIGNAL_MATCH_DATA, 0, 0, 0, 0, this);
        hide();
    }
}

void PopupMenu::show(const IntRect& rect, FrameView* view, int index)
{
    ASSERT(client());

    if (!m_popup) {
        m_popup = GTK_MENU(gtk_menu_new());
        g_signal_connect(m_popup.get(), "unmap", G_CALLBACK(menuUnmapped), this);
    } else
        gtk_container_foreach(GTK_CONTAINER(m_popup.get()), reinterpret_cast<GtkCallback>(menuRemoveItem), this);

    int x, y;
    gdk_window_get_origin(GTK_WIDGET(view->hostWindow()->platformPageClient())->window, &x, &y);
    m_menuPosition = view->contentsToWindow(rect.location());
    m_menuPosition = IntPoint(m_menuPosition.x() + x, m_menuPosition.y() + y + rect.height());
    m_indexMap.clear();

    const int size = client()->listSize();
    for (int i = 0; i < size; ++i) {
        GtkWidget* item;
        if (client()->itemIsSeparator(i))
            item = gtk_separator_menu_item_new();
        else
            item = gtk_menu_item_new_with_label(client()->itemText(i).utf8().data());

        m_indexMap.add(item, i);
        g_signal_connect(item, "activate", G_CALLBACK(menuItemActivated), this);

        // FIXME: Apply the PopupMenuStyle from client()->itemStyle(i)
        gtk_widget_set_sensitive(item, client()->itemIsEnabled(i));
        gtk_menu_shell_append(GTK_MENU_SHELL(m_popup.get()), item);
        gtk_widget_show(item);
    }

    gtk_menu_set_active(m_popup.get(), index);


    // The size calls are directly copied from gtkcombobox.c which is LGPL
    GtkRequisition requisition;
    gtk_widget_set_size_request(GTK_WIDGET(m_popup.get()), -1, -1);
    gtk_widget_size_request(GTK_WIDGET(m_popup.get()), &requisition);
    gtk_widget_set_size_request(GTK_WIDGET(m_popup.get()), std::max(rect.width(), requisition.width), -1);

    GList* children = GTK_MENU_SHELL(m_popup.get())->children;
    if (size)
        for (int i = 0; i < size; i++) {
            if (i > index)
              break;

            GtkWidget* item = reinterpret_cast<GtkWidget*>(children->data);
            GtkRequisition itemRequisition;
            gtk_widget_get_child_requisition(item, &itemRequisition);
            m_menuPosition.setY(m_menuPosition.y() - itemRequisition.height);

            children = g_list_next(children);
        } else
            // Center vertically the empty popup in the combo box area
            m_menuPosition.setY(m_menuPosition.y() - rect.height() / 2);

    gtk_menu_popup(m_popup.get(), 0, 0, reinterpret_cast<GtkMenuPositionFunc>(menuPositionFunction), this, 0, gtk_get_current_event_time());
}

void PopupMenu::hide()
{
    ASSERT(m_popup);
    gtk_menu_popdown(m_popup.get());
}

void PopupMenu::updateFromElement()
{
    client()->setTextFromItem(client()->selectedIndex());
}

bool PopupMenu::itemWritingDirectionIsNatural()
{
    return true;
}

void PopupMenu::menuItemActivated(GtkMenuItem* item, PopupMenu* that)
{
    ASSERT(that->client());
    ASSERT(that->m_indexMap.contains(GTK_WIDGET(item)));
    that->client()->valueChanged(that->m_indexMap.get(GTK_WIDGET(item)));
}

void PopupMenu::menuUnmapped(GtkWidget*, PopupMenu* that)
{
    ASSERT(that->client());
    that->client()->popupDidHide();
}

void PopupMenu::menuPositionFunction(GtkMenu*, gint* x, gint* y, gboolean* pushIn, PopupMenu* that)
{
    *x = that->m_menuPosition.x();
    *y = that->m_menuPosition.y();
    *pushIn = true;
}

void PopupMenu::menuRemoveItem(GtkWidget* widget, PopupMenu* that)
{
    ASSERT(that->m_popup);
    gtk_container_remove(GTK_CONTAINER(that->m_popup.get()), widget);
}

}

