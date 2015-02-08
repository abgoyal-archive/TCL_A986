


#include "config.h"

#include <wtf/CurrentTime.h>

#include <QBasicTimer>
#include <QCoreApplication>
#include <QDebug>
#include <QPointer>

namespace WebCore {

class SharedTimerQt : public QObject {
    friend void setSharedTimerFiredFunction(void (*f)());
public:
    static SharedTimerQt* inst();

    void start(double);
    void stop();

protected:
    void timerEvent(QTimerEvent* ev);

private:
    SharedTimerQt(QObject* parent);
    ~SharedTimerQt();
    QBasicTimer m_timer;
    void (*m_timerFunction)();
};

SharedTimerQt::SharedTimerQt(QObject* parent)
    : QObject(parent)
    , m_timerFunction(0)
{}

SharedTimerQt::~SharedTimerQt()
{
    if (m_timer.isActive())
        (m_timerFunction)();
}

SharedTimerQt* SharedTimerQt::inst()
{
    static QPointer<SharedTimerQt> timer;
    if (!timer)
        timer = new SharedTimerQt(QCoreApplication::instance());

    return timer;
}

void SharedTimerQt::start(double fireTime)
{
    double interval = fireTime - currentTime();
    unsigned int intervalInMS;
    if (interval < 0)
        intervalInMS = 0;
    else {
        interval *= 1000;
        intervalInMS = (unsigned int)interval;
    }

    m_timer.start(intervalInMS, this);
}

void SharedTimerQt::stop()
{
    m_timer.stop();
}

void SharedTimerQt::timerEvent(QTimerEvent* ev)
{
    if (!m_timerFunction || ev->timerId() != m_timer.timerId())
        return;

    m_timer.stop();
    (m_timerFunction)();
}

void setSharedTimerFiredFunction(void (*f)())
{
    if (!QCoreApplication::instance())
        return;

    SharedTimerQt::inst()->m_timerFunction = f;
}

void setSharedTimerFireTime(double fireTime)
{
    if (!QCoreApplication::instance())
        return;

    SharedTimerQt::inst()->start(fireTime);
}

void stopSharedTimer()
{
    if (!QCoreApplication::instance())
        return;

    SharedTimerQt::inst()->stop();
}

}

// vim: ts=4 sw=4 et
