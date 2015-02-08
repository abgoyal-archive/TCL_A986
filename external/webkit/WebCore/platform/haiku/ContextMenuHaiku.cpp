

#include "config.h"
#include "ContextMenu.h"

#include "ContextMenuController.h"
#include "ContextMenuItem.h"
#include "Document.h"
#include "Frame.h"
#include "FrameView.h"
#include <Looper.h>
#include <Menu.h>
#include <Message.h>
#include <wtf/Assertions.h>


namespace WebCore {

// FIXME: This class isn't used yet
class ContextMenuReceiver : public BLooper {
public:
    ContextMenuReceiver(ContextMenu* menu)
        : BLooper("context_menu_receiver")
        , m_menu(menu)
        , m_result(-1)
    {
    }

    void HandleMessage(BMessage* msg)
    {
        m_result = msg->what;
        if (m_result != -1) {
            BMenuItem* item = m_menu->platformDescription()->FindItem(m_result);
            if (!item) {
                printf("Error: Context menu item with code %i not found!\n", m_result);
                return;
            }
            ContextMenuItem cmItem(item);
            m_menu->controller()->contextMenuItemSelected(&cmItem);
            cmItem.releasePlatformDescription();
        }
    }

    int Result()
    {
        return m_result;
    }

private:
    ContextMenu* m_menu;
    int m_result;
};

ContextMenu::ContextMenu(const HitTestResult& result)
    : m_hitTestResult(result)
    , m_platformDescription(new BMenu("context_menu"))
{
}

ContextMenu::~ContextMenu()
{
    delete m_platformDescription;
}

void ContextMenu::appendItem(ContextMenuItem& item)
{
    checkOrEnableIfNeeded(item);

    BMenuItem* menuItem = item.releasePlatformDescription();
    if (menuItem)
        m_platformDescription->AddItem(menuItem);
}

unsigned ContextMenu::itemCount() const
{
    return m_platformDescription->CountItems();
}

void ContextMenu::insertItem(unsigned position, ContextMenuItem& item)
{
    checkOrEnableIfNeeded(item);

    BMenuItem* menuItem = item.releasePlatformDescription();
    if (menuItem)
        m_platformDescription->AddItem(menuItem, position);
}

PlatformMenuDescription ContextMenu::platformDescription() const
{
    return m_platformDescription;
}

void ContextMenu::setPlatformDescription(PlatformMenuDescription menu)
{
    if (static_cast<BMenu*>(menu) == m_platformDescription)
        return;

    delete m_platformDescription;
    m_platformDescription = static_cast<BMenu*>(menu);
}

} // namespace WebCore

