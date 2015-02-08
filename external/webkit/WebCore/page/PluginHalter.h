

#ifndef PluginHalter_h
#define PluginHalter_h

#include "PluginHalterClient.h"
#include "Timer.h"
#include <wtf/HashMap.h>
#include <wtf/OwnPtr.h>

namespace WebCore {

class HaltablePlugin;

class PluginHalter : public Noncopyable {
public:
    PluginHalter(PluginHalterClient*);

    void didStartPlugin(HaltablePlugin*);
    void didStopPlugin(HaltablePlugin*);

    void setPluginAllowedRunTime(unsigned runTime) { m_pluginAllowedRunTime = runTime; }

private:
    void timerFired(Timer<PluginHalter>*);
    void startTimerIfNecessary();

    OwnPtr<PluginHalterClient> m_client;
    Timer<PluginHalter> m_timer;
    unsigned m_pluginAllowedRunTime;
    double m_oldestStartTime;
    HashMap<HaltablePlugin*, double> m_plugins;
};

} // namespace WebCore

#endif // PluginHalter_h
