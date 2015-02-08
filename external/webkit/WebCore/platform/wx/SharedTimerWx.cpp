

#include "config.h"

#include "SharedTimer.h"
#include "Widget.h"

#include <wtf/Assertions.h>
#include <wtf/CurrentTime.h>
#include <stdio.h>

#include "wx/defs.h"
#include "wx/timer.h"

namespace WebCore {

static void (*sharedTimerFiredFunction)();

class WebKitTimer: public wxTimer
{
    public:
        WebKitTimer();
        ~WebKitTimer();
        virtual void Notify();
};

WebKitTimer::WebKitTimer()
{
    wxTimer::wxTimer();
}

WebKitTimer::~WebKitTimer()
{
}

void WebKitTimer::Notify()
{
    sharedTimerFiredFunction();
}

static WebKitTimer* wkTimer; 

void setSharedTimerFiredFunction(void (*f)())
{
    sharedTimerFiredFunction = f;
}

void setSharedTimerFireTime(double fireTime)
{
    ASSERT(sharedTimerFiredFunction);
    
    double interval = fireTime - currentTime();
    
    if (!wkTimer)
        wkTimer = new WebKitTimer();
        
    int intervalInMS = interval * 1000;

    // sanity check
    if (intervalInMS < 1)
        intervalInMS = 1;

    wkTimer->Start(intervalInMS, wxTIMER_ONE_SHOT);
}

void stopSharedTimer()
{
    if (wkTimer)
        wkTimer->Stop();
}

}
