

#include "config.h"
#include "SharedTimer.h"

#include <MessageFilter.h>
#include <MessageRunner.h>
#include <Looper.h>
#include <support/Locker.h>
#include <support/Autolock.h>
#include <wtf/CurrentTime.h>

#define FIRE_MESSAGE 'fire'


namespace WebCore {

class SharedTimerHaiku: public BMessageFilter {
    friend void setSharedTimerFiredFunction(void (*f)());
public:
    static SharedTimerHaiku* instance();

    void start(double);
    void stop();

protected:
    virtual filter_result Filter(BMessage*, BHandler**);

private:
    SharedTimerHaiku();
    ~SharedTimerHaiku();

    void (*m_timerFunction)();
    bool m_shouldRun;
};

SharedTimerHaiku::SharedTimerHaiku()
    : BMessageFilter(FIRE_MESSAGE)
    , m_timerFunction(0)
    , m_shouldRun(false)
{
}

SharedTimerHaiku::~SharedTimerHaiku()
{
}

SharedTimerHaiku* SharedTimerHaiku::instance()
{
    BLooper* looper = BLooper::LooperForThread(find_thread(0));
    static SharedTimerHaiku* timer;

    if (!timer) {
        BAutolock lock(looper);
        timer = new SharedTimerHaiku();
        looper->AddCommonFilter(timer);
    }

    return timer;
}

void SharedTimerHaiku::start(double fireTime)
{
    m_shouldRun = true;

    double intervalInSeconds = fireTime - currentTime();
    bigtime_t intervalInMicroSeconds = intervalInSeconds < 0 ? 0 : intervalInSeconds * 1000000;

    BMessageRunner::StartSending(Looper(), new BMessage(FIRE_MESSAGE), intervalInMicroSeconds, 1);
}

void SharedTimerHaiku::stop()
{
    m_shouldRun = false;
}

filter_result SharedTimerHaiku::Filter(BMessage*, BHandler**)
{
    if (m_shouldRun && m_timerFunction)
        m_timerFunction();

    return B_SKIP_MESSAGE;
}

// WebCore functions
void setSharedTimerFiredFunction(void (*f)())
{
    SharedTimerHaiku::instance()->m_timerFunction = f;
}

void setSharedTimerFireTime(double fireTime)
{
    SharedTimerHaiku::instance()->start(fireTime);
}

void stopSharedTimer()
{
    SharedTimerHaiku::instance()->stop();
}

} // namespace WebCore
