
#ifndef QtAbstractWebPopup_h
#define QtAbstractWebPopup_h

#include "PopupMenuClient.h"

#include <QFont>
#include <QList>
#include <QRect>
#include <QWidget>

class QWebPageClient;

namespace WebCore {

class QtAbstractWebPopup {
public:
    enum ItemType { Option, Group, Separator };

    ItemType itemType(int) const;
    QString itemText(int idx) const { return m_popupClient->itemText(idx); }
    QString itemToolTip(int idx) const { return m_popupClient->itemToolTip(idx); }
    bool itemIsEnabled(int idx) const { return m_popupClient->itemIsEnabled(idx); }
    int itemCount() const { return m_popupClient->listSize(); }

    QWebPageClient* pageClient() const { return m_pageClient; }
    QRect geometry() const { return m_geometry; }
    int currentIndex() const { return m_currentIndex; }

    QtAbstractWebPopup();
    virtual ~QtAbstractWebPopup();

    virtual void show() = 0;
    virtual void hide() = 0;

    void popupDidHide();
    void valueChanged(int index);

    QFont font() { return m_popupClient->menuStyle().font().font(); }

private:
    friend class PopupMenu;
    PopupMenuClient* m_popupClient;
    QWebPageClient* m_pageClient;
    int m_currentIndex;
    QRect m_geometry;
};

}

#endif // QtAbstractWebPopup_h
