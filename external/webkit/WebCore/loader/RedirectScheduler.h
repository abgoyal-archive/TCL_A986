

#ifndef RedirectScheduler_h
#define RedirectScheduler_h

#include "Event.h"
#include "Timer.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class FormState;
class Frame;
class String;

struct FrameLoadRequest;
struct ScheduledRedirection;

class RedirectScheduler : public Noncopyable {
public:
    RedirectScheduler(Frame*);
    ~RedirectScheduler();

    bool redirectScheduledDuringLoad();
    bool locationChangePending();

    void scheduleRedirect(double delay, const String& url);
    void scheduleLocationChange(const String& url, const String& referrer, bool lockHistory = true, bool lockBackForwardList = true, bool userGesture = false);
    void scheduleFormSubmission(const FrameLoadRequest&, bool lockHistory, PassRefPtr<Event>, PassRefPtr<FormState>);
    void scheduleRefresh(bool userGesture = false);
    void scheduleHistoryNavigation(int steps);

    void startTimer();

    void cancel(bool newLoadInProgress = false);
    void clear();

private:
    void timerFired(Timer<RedirectScheduler>*);
    void schedule(PassOwnPtr<ScheduledRedirection>);

    static bool mustLockBackForwardList(Frame* targetFrame);

    Frame* m_frame;
    Timer<RedirectScheduler> m_timer;
    OwnPtr<ScheduledRedirection> m_scheduledRedirection;
};

} // namespace WebCore

#endif // FrameLoader_h
