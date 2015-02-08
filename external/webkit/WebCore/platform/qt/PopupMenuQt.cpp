

#include "config.h"
#include "PopupMenu.h"

#include "Chrome.h"
#include "ChromeClientQt.h"
#include "FrameView.h"
#include "PopupMenuClient.h"
#include "QWebPageClient.h"
#include "QtAbstractWebPopup.h"

namespace WebCore {

PopupMenu::PopupMenu(PopupMenuClient* client)
    : m_popupClient(client)
    , m_popup(0)
{
}

PopupMenu::~PopupMenu()
{
    delete m_popup;
}

void PopupMenu::show(const IntRect& rect, FrameView* view, int index)
{
    ChromeClientQt* chromeClient = static_cast<ChromeClientQt*>(
        view->frame()->page()->chrome()->client());
    ASSERT(chromeClient);

    if (!m_popup)
        m_popup = chromeClient->createSelectPopup();

    m_popup->m_popupClient = m_popupClient;
    m_popup->m_currentIndex = index;
    m_popup->m_pageClient = chromeClient->platformPageClient();

    QRect geometry(rect);
    geometry.moveTopLeft(view->contentsToWindow(rect.topLeft()));
    m_popup->m_geometry = geometry;

    m_popup->show();

}

void PopupMenu::hide()
{
    m_popup->hide();
}

void PopupMenu::updateFromElement()
{
    client()->setTextFromItem(m_popupClient->selectedIndex());
}

bool PopupMenu::itemWritingDirectionIsNatural()
{
    return false;
}

}

// vim: ts=4 sw=4 et
