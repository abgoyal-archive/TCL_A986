
#include "config.h"
#include "QtAbstractWebPopup.h"

#include "PopupMenuClient.h"


namespace WebCore {

QtAbstractWebPopup::QtAbstractWebPopup()
    : m_popupClient(0)
    , m_pageClient(0)
    , m_currentIndex(-1)
{
}

QtAbstractWebPopup::~QtAbstractWebPopup()
{
}

void QtAbstractWebPopup::popupDidHide()
{
    Q_ASSERT(m_popupClient);
    m_popupClient->popupDidHide();
}

void QtAbstractWebPopup::valueChanged(int index)
{
    Q_ASSERT(m_popupClient);
    m_popupClient->valueChanged(index);
}

QtAbstractWebPopup::ItemType QtAbstractWebPopup::itemType(int idx) const
{
    if (m_popupClient->itemIsSeparator(idx))
        return Separator;
    if (m_popupClient->itemIsLabel(idx))
        return Group;
    return Option;
}

} // namespace WebCore
