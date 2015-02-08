
#ifndef EventSenderQt_h
#define EventSenderQt_h

#include <QApplication>
#include <QEvent>
#include <QEventLoop>
#include <QMouseEvent>
#include <QObject>
#include <QPoint>
#include <QString>
#include <QStringList>

#include <qwebpage.h>
#include <qwebframe.h>

#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
#include <QTouchEvent>
#endif

class EventSender : public QObject {
    Q_OBJECT
public:
    EventSender(QWebPage* parent);
    virtual bool eventFilter(QObject* watched, QEvent* event);

public slots:
    void mouseDown(int button = 0);
    void mouseUp(int button = 0);
    void mouseMoveTo(int x, int y);
    void leapForward(int ms);
    void keyDown(const QString& string, const QStringList& modifiers = QStringList(), unsigned int location = 0);
    void clearKillRing() {}
    void contextClick();
    void scheduleAsynchronousClick();
    void addTouchPoint(int x, int y);
    void updateTouchPoint(int index, int x, int y);
    void setTouchModifier(const QString &modifier, bool enable);
    void touchStart();
    void touchMove();
    void touchEnd();
    void zoomPageIn();
    void zoomPageOut();
    void clearTouchPoints();
    void releaseTouchPoint(int index);

private:
    void sendTouchEvent(QEvent::Type);
    void sendOrQueueEvent(QEvent*);
    void replaySavedEvents(bool flush);
    QPoint m_mousePos;
    Qt::MouseButtons m_mouseButtons;
    QWebPage* m_page;
    int m_timeLeap;
    bool m_mouseButtonPressed;
    bool m_drag;
    QEventLoop* m_eventLoop;
    QWebFrame* frameUnderMouse() const;
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    QList<QTouchEvent::TouchPoint> m_touchPoints;
    Qt::KeyboardModifiers m_touchModifiers;
    bool m_touchActive;
#endif
};
#endif //  EventSenderQt_h
