

#ifndef QWEBINSPECTOR_P_H
#define QWEBINSPECTOR_P_H

QT_BEGIN_NAMESPACE
class QSize;
class QWidget;
QT_END_NAMESPACE
class QWebInspector;
class QWebPage;

class QWebInspectorPrivate {
public:
    QWebInspectorPrivate(QWebInspector* qq)
    : q(qq)
    , page(0)
    , frontend(0)
    {}

    void setFrontend(QWidget* newFrontend);
    void adjustFrontendSize(const QSize& size);

    QWebInspector* q;
    QWebPage* page;
    QWidget* frontend;
};

#endif
