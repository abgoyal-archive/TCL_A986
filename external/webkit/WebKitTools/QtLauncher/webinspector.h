

#ifndef webinspector_h
#define webinspector_h

#include <QtGui>
#include "qwebinspector.h"

class WebInspector : public QWebInspector {
    Q_OBJECT

public:
    WebInspector(QWidget* parent) : QWebInspector(parent) {}

signals:
    void visibleChanged(bool nowVisible);

protected:
    void showEvent(QShowEvent* event)
    {
        QWebInspector::showEvent(event);
        emit visibleChanged(true);
    }
    void hideEvent(QHideEvent* event)
    {
        QWebInspector::hideEvent(event);
        emit visibleChanged(false);
    }
};

#endif
