

#include "config.h"
#include "ContextMenu.h"

#include <wtf/Assertions.h>

#include <QAction>

#include <Document.h>
#include <Frame.h>
#include <FrameView.h>

namespace WebCore {

ContextMenu::ContextMenu(const HitTestResult& result)
    : m_hitTestResult(result)
{
}

ContextMenu::~ContextMenu()
{
}

void ContextMenu::appendItem(ContextMenuItem& item)
{
    m_items.append(item);
}

unsigned ContextMenu::itemCount() const
{
    return m_items.count();
}

void ContextMenu::insertItem(unsigned position, ContextMenuItem& item)
{
    m_items.insert(position, item);
}

void ContextMenu::setPlatformDescription(PlatformMenuDescription)
{
    // doesn't make sense
}

PlatformMenuDescription ContextMenu::platformDescription() const
{
    return &m_items;
}

PlatformMenuDescription ContextMenu::releasePlatformDescription()
{
    return PlatformMenuDescription();
}


}
// vim: ts=4 sw=4 et
