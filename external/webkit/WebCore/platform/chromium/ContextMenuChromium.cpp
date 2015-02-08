

#include "config.h"
#include "ContextMenu.h"

namespace WebCore {

// This is a stub implementation of WebKit's ContextMenu class that does
// nothing.

ContextMenu::ContextMenu(const HitTestResult& result)
    : m_hitTestResult(result)
{
}

ContextMenu::ContextMenu(const HitTestResult& result, const PlatformMenuDescription menu)
    : m_hitTestResult(result)
{
}

ContextMenu::~ContextMenu()
{
}

unsigned ContextMenu::itemCount() const
{
    return m_items.size();
}

void ContextMenu::insertItem(unsigned position, ContextMenuItem& item)
{
    m_items.insert(position, item);
}

void ContextMenu::appendItem(ContextMenuItem& item)
{
    m_items.append(item);
}

ContextMenuItem* ContextMenu::itemWithAction(unsigned action)
{
    for (size_t i = 0; i < m_items.size(); ++i) {
        if (m_items[i].action() == static_cast<ContextMenuAction>(action))
            return &m_items[i];
    }
    return 0;
}

ContextMenuItem* ContextMenu::itemAtIndex(unsigned index, const PlatformMenuDescription platformDescription)
{
    return &m_items[index];
}

void ContextMenu::setPlatformDescription(PlatformMenuDescription menu)
{
}

PlatformMenuDescription ContextMenu::platformDescription() const
{
    return 0;
}

PlatformMenuDescription ContextMenu::releasePlatformDescription()
{
    return 0;
}

} // namespace WebCore
