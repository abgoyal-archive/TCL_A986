

#include "config.h"
#include "ContextMenuItem.h"

namespace WebCore {

// This is a stub implementation of WebKit's ContextMenu class that does
// nothing.

ContextMenuItem::ContextMenuItem(PlatformMenuItemDescription item)
{
}

ContextMenuItem::ContextMenuItem(ContextMenu* subMenu)
{
}

ContextMenuItem::ContextMenuItem(ContextMenuItemType type, ContextMenuAction action, const String& title, ContextMenu* subMenu)
{
    m_platformDescription.type = type;
    m_platformDescription.action = action;
    m_platformDescription.title = title;
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

ContextMenuAction ContextMenuItem::action() const
{ 
    return m_platformDescription.action;
}

String ContextMenuItem::title() const 
{
    return m_platformDescription.title;
}

bool ContextMenuItem::checked() const
{
    return m_platformDescription.checked;
}

bool ContextMenuItem::enabled() const
{
    return m_platformDescription.enabled;
}

PlatformMenuDescription ContextMenuItem::platformSubMenu() const
{
    return PlatformMenuDescription();
}

void ContextMenuItem::setType(ContextMenuItemType type)
{
    m_platformDescription.type = type;
}

void ContextMenuItem::setAction(ContextMenuAction action)
{
    m_platformDescription.action = action;
}

void ContextMenuItem::setTitle(const String& title)
{
    m_platformDescription.title = title;
}

void ContextMenuItem::setSubMenu(ContextMenu* subMenu)
{
}

void ContextMenuItem::setChecked(bool checked)
{
    m_platformDescription.checked = checked;
}

void ContextMenuItem::setEnabled(bool enabled)
{
    m_platformDescription.enabled = enabled;
}

} // namespace WebCore
