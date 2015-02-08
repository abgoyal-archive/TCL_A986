

#include "config.h"

#include "ContextMenuItem.h"
#include "ContextMenu.h"
#include "PlatformMenuDescription.h"

using namespace WebCore;

ContextMenuItem::ContextMenuItem(ContextMenu* subMenu)
{
    m_platformDescription.type = SubmenuType;
    m_platformDescription.action = ContextMenuItemTagNoAction;
    if (subMenu)
        setSubMenu(subMenu);
    else
        m_platformDescription.subMenu = 0;
}

ContextMenuItem::ContextMenuItem(ContextMenuItemType type, ContextMenuAction action,
                                 const String& title, ContextMenu* subMenu)
{
    m_platformDescription.type = type;
    m_platformDescription.action = action;
    m_platformDescription.title = title;
    if (subMenu)
        setSubMenu(subMenu);
    else
        m_platformDescription.subMenu = 0;
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

void ContextMenuItem::setSubMenu(ContextMenu* subMenu)
{
    m_platformDescription.subMenu = subMenu->releasePlatformDescription();
}

void ContextMenuItem::setChecked(bool shouldCheck)
{
    m_platformDescription.checked = shouldCheck;
}

void ContextMenuItem::setEnabled(bool shouldEnable)
{
    m_platformDescription.enabled = shouldEnable;
}
