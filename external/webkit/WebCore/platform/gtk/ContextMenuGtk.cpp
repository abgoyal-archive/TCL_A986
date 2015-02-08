

#include "config.h"
#include "ContextMenu.h"

#include "ContextMenuController.h"

#include <gtk/gtk.h>

namespace WebCore {

// TODO: ref-counting correctness checking.
// See http://bugs.webkit.org/show_bug.cgi?id=16115

static void menuItemActivated(GtkMenuItem* item, ContextMenuController* controller)
{
    ContextMenuItem contextItem(item);
    controller->contextMenuItemSelected(&contextItem);
}

ContextMenu::ContextMenu(const HitTestResult& result)
    : m_hitTestResult(result)
{
    m_platformDescription = GTK_MENU(gtk_menu_new());

    g_object_ref_sink(G_OBJECT(m_platformDescription));
}

ContextMenu::~ContextMenu()
{
    if (m_platformDescription)
        g_object_unref(m_platformDescription);
}

void ContextMenu::appendItem(ContextMenuItem& item)
{
    ASSERT(m_platformDescription);
    checkOrEnableIfNeeded(item);

    ContextMenuItemType type = item.type();
    GtkMenuItem* platformItem = ContextMenuItem::createNativeMenuItem(item.releasePlatformDescription());
    ASSERT(platformItem);

    if (type == ActionType || type == CheckableActionType)
        g_signal_connect(platformItem, "activate", G_CALLBACK(menuItemActivated), controller());

    gtk_menu_shell_append(GTK_MENU_SHELL(m_platformDescription), GTK_WIDGET(platformItem));
    gtk_widget_show(GTK_WIDGET(platformItem));
}

void ContextMenu::setPlatformDescription(PlatformMenuDescription menu)
{
    ASSERT(menu);
    if (m_platformDescription)
        g_object_unref(m_platformDescription);

    m_platformDescription = menu;
    g_object_ref(m_platformDescription);
}

PlatformMenuDescription ContextMenu::platformDescription() const
{
    return m_platformDescription;
}

PlatformMenuDescription ContextMenu::releasePlatformDescription()
{
    PlatformMenuDescription description = m_platformDescription;
    m_platformDescription = 0;

    return description;
}

}
