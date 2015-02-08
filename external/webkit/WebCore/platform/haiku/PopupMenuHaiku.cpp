

#include "config.h"
#include "PopupMenu.h"

#include "FrameView.h"

#include "NotImplemented.h"
#include <Menu.h>


namespace WebCore {

PopupMenu::PopupMenu(PopupMenuClient* client)
    : m_popupClient(client)
    , m_menu(0)
{
}

PopupMenu::~PopupMenu()
{
    delete m_menu;
    m_menu = 0;
}

void PopupMenu::show(const IntRect& rect, FrameView* view, int index)
{
    notImplemented();
}

void PopupMenu::hide()
{
    notImplemented();
}

void PopupMenu::updateFromElement()
{
    notImplemented();
}

bool PopupMenu::itemWritingDirectionIsNatural()
{
    notImplemented();
    return true;
}

} // namespace WebCore

