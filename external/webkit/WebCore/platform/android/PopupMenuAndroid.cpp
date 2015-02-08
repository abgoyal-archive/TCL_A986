

#include "config.h"
#include "PopupMenu.h"

namespace WebCore {

// Now we handle all of this in WebViewCore.cpp.
PopupMenu::PopupMenu(PopupMenuClient* menuList)
    : m_popupClient(menuList)
{
}

PopupMenu::~PopupMenu()
{
}

void PopupMenu::show(const IntRect&, FrameView*, int)
{
}

void PopupMenu::hide()
{
}

void PopupMenu::updateFromElement() 
{
}

bool PopupMenu::itemWritingDirectionIsNatural()
{
    return false;
}

} // namespace WebCore
