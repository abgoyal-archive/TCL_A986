
#ifndef QtFallbackWebPopup_h
#define QtFallbackWebPopup_h

#include "QtAbstractWebPopup.h"
#include <QComboBox>

class QGraphicsProxyWidget;

namespace WebCore {

class QtFallbackWebPopupCombo;

class QtFallbackWebPopup : public QObject, public QtAbstractWebPopup {
    Q_OBJECT
public:
    QtFallbackWebPopup();
    ~QtFallbackWebPopup();

    virtual void show();
    virtual void hide();

private slots:
    void activeChanged(int);

private:
    friend class QtFallbackWebPopupCombo;
    bool m_popupVisible;
    QtFallbackWebPopupCombo* m_combo;
    QGraphicsProxyWidget* m_proxy;

    void populate();
};

class QtFallbackWebPopupCombo : public QComboBox {
public:
    QtFallbackWebPopupCombo(QtFallbackWebPopup& ownerPopup);
    virtual void showPopup();
    virtual void hidePopup();

private:
    QtFallbackWebPopup& m_ownerPopup;
};

}

#endif // QtFallbackWebPopup_h
