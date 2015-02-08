

#include "config.h"
#include "ContextMenuItem.h"
#include "ContextMenu.h"

namespace WebCore {

ContextMenuItem::ContextMenuItem(ContextMenu* subMenu)
{
    m_platformDescription.type = SubmenuType;
    m_platformDescription.action = ContextMenuItemTagNoAction;
    if (subMenu)
        setSubMenu(subMenu);
}

ContextMenuItem::ContextMenuItem(ContextMenuItemType type, ContextMenuAction action,
                                 const String& title, ContextMenu* subMenu)
{
    m_platformDescription.type = type;
    m_platformDescription.action = action;
    m_platformDescription.title = title;
    if (subMenu)
        setSubMenu(subMenu);
}

ContextMenuItem::~ContextMenuItem()
{
}

PlatformMenuItemDescription ContextMenuItem::releasePlatformDescription()
{
    return m_platformDescription;
}

ContextMenuItemType ContextMenuItem::type() const
{
    return m_platformDescription.type;
}

void ContextMenuItem::setType(ContextMenuItemType type)
{
    m_platformDescription.type = type;
}

ContextMenuAction ContextMenuItem::action() const
{
    return m_platformDescription.action;
}

void ContextMenuItem::setAction(ContextMenuAction action)
{
    m_platformDescription.action = action;
}

String ContextMenuItem::title() const
{
    return m_platformDescription.title;
}

void ContextMenuItem::setTitle(const String& title)
{
    m_platformDescription.title = title;
}


PlatformMenuDescription ContextMenuItem::platformSubMenu() const
{
    return &m_platformDescription.subMenuItems;
}

void ContextMenuItem::setSubMenu(ContextMenu* menu)
{
    m_platformDescription.subMenuItems = *menu->platformDescription();
}

void ContextMenuItem::setChecked(bool on)
{
    m_platformDescription.checked = on;
}

void ContextMenuItem::setEnabled(bool on)
{
    m_platformDescription.enabled = on;
}

bool ContextMenuItem::enabled() const
{
    return m_platformDescription.enabled;
}

}
// vim: ts=4 sw=4 et
