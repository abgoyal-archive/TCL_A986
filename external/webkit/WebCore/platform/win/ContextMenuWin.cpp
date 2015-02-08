

#include "config.h"
#include "ContextMenu.h"

#include "CString.h"
#include "Document.h"
#include "Frame.h"
#include "FrameView.h"
#include "Node.h"
#include <tchar.h>
#include <windows.h>

namespace WebCore {

ContextMenu::ContextMenu(const HitTestResult& result)
    : m_hitTestResult(result)
    , m_platformDescription(0)
{
    setPlatformDescription(::CreatePopupMenu());
}

ContextMenu::ContextMenu(const HitTestResult& result, const PlatformMenuDescription menu)
    : m_hitTestResult(result)
    , m_platformDescription(0)
{
    setPlatformDescription(menu);
}

ContextMenu::~ContextMenu()
{
    if (m_platformDescription)
        ::DestroyMenu(m_platformDescription);
}

unsigned ContextMenu::itemCount() const
{
    if (!m_platformDescription)
        return 0;

    return ::GetMenuItemCount(m_platformDescription);
}

void ContextMenu::insertItem(unsigned int position, ContextMenuItem& item)
{
    if (!m_platformDescription)
        return;

    checkOrEnableIfNeeded(item);
    ::InsertMenuItem(m_platformDescription, position, TRUE, item.releasePlatformDescription());
}

void ContextMenu::appendItem(ContextMenuItem& item)
{
    insertItem(itemCount(), item);
}

static ContextMenuItem* contextMenuItemByIdOrPosition(HMENU menu, unsigned id, BOOL byPosition)
{
    if (!menu)
        return 0;
    LPMENUITEMINFO info = (LPMENUITEMINFO)malloc(sizeof(MENUITEMINFO));
    if (!info)
        return 0;

    memset(info, 0, sizeof(MENUITEMINFO));

    info->cbSize = sizeof(MENUITEMINFO);
    
    info->fMask = MIIM_FTYPE | MIIM_ID | MIIM_STRING;

    if (!::GetMenuItemInfo(menu, id, byPosition, info)) {
        free(info);
        return 0;
    }

    UINT type = info->fType & ~(MFT_MENUBARBREAK | MFT_MENUBREAK | MFT_OWNERDRAW | MFT_RADIOCHECK | MFT_RIGHTORDER | MFT_RIGHTJUSTIFY);
    if (type == MFT_STRING) {
        LPTSTR buffer = (LPTSTR)malloc(++info->cch * sizeof(TCHAR));
        if (!buffer) {
            free(info);
            return 0;
        }
        info->dwTypeData = buffer;
        ::GetMenuItemInfo(menu, id, byPosition, info);
    }
    
    return new ContextMenuItem(info);
}

ContextMenuItem* ContextMenu::itemWithAction(unsigned action)
{
    return contextMenuItemByIdOrPosition(m_platformDescription, action, FALSE);
}

ContextMenuItem* ContextMenu::itemAtIndex(unsigned index, const PlatformMenuDescription platformDescription)
{
    return contextMenuItemByIdOrPosition(platformDescription, index, TRUE);
}

void ContextMenu::setPlatformDescription(HMENU menu)
{
    if (menu == m_platformDescription)
        return;
    
    if (m_platformDescription)
        ::DestroyMenu(m_platformDescription);

    m_platformDescription = menu;
    if (!m_platformDescription)
        return;

    MENUINFO menuInfo = {0};
    menuInfo.cbSize = sizeof(MENUINFO);
    menuInfo.fMask = MIM_STYLE;
    ::GetMenuInfo(m_platformDescription, &menuInfo);
    menuInfo.fMask = MIM_STYLE;
    menuInfo.dwStyle |= MNS_NOTIFYBYPOS;
    ::SetMenuInfo(m_platformDescription, &menuInfo);
}

HMENU ContextMenu::platformDescription() const
{
    return m_platformDescription;
}

HMENU ContextMenu::releasePlatformDescription()
{
    HMENU description = m_platformDescription;
    m_platformDescription = 0;
    return description;
}

}
