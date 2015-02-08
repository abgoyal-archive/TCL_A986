

#include "config.h"

#include "PluginHalter.h"

#include "HaltablePlugin.h"
#include "PlatformString.h"
#include <wtf/CurrentTime.h>
#include <wtf/Vector.h>

using namespace std;

namespace WebCore {

PluginHalter::PluginHalter(PluginHalterClient* client)
    : m_client(client)
    , m_timer(this, &PluginHalter::timerFired)
    , m_pluginAllowedRunTime(numeric_limits<unsigned>::max())
{
    ASSERT_ARG(client, client);
}

void PluginHalter::didStartPlugin(HaltablePlugin* obj)
{
    ASSERT_ARG(obj, obj);
    ASSERT_ARG(obj, !m_plugins.contains(obj));

    if (!m_client->enabled())
        return;

    double currentTime = WTF::currentTime();

    m_plugins.add(obj, currentTime);

    if (m_plugins.size() == 1)
        m_oldestStartTime = currentTime;

    startTimerIfNecessary();
}

void PluginHalter::didStopPlugin(HaltablePlugin* obj)
{
    if (!m_client->enabled())
        return;

    m_plugins.remove(obj);
}

void PluginHalter::timerFired(Timer<PluginHalter>*)
{
    if (m_plugins.isEmpty())
        return;

    Vector<HaltablePlugin*> plugins;
    copyKeysToVector(m_plugins, plugins);

    // Plug-ins older than this are candidates to be halted.
    double pluginCutOffTime = WTF::currentTime() - m_pluginAllowedRunTime;

    m_oldestStartTime = numeric_limits<double>::max();

    for (size_t i = 0; i < plugins.size(); ++i) {
        double thisStartTime = m_plugins.get(plugins[i]);
        if (thisStartTime > pluginCutOffTime) {
            // This plug-in is too young to be halted. We find the oldest
            // plug-in that is not old enough to be halted and use it to set
            // the timer's next fire time.
            if (thisStartTime < m_oldestStartTime)
                m_oldestStartTime = thisStartTime;
            continue;
        }

        if (m_client->shouldHaltPlugin(plugins[i]->node(), plugins[i]->isWindowed(), plugins[i]->pluginName()))
            plugins[i]->halt();

        m_plugins.remove(plugins[i]);
    }

    startTimerIfNecessary();
}

void PluginHalter::startTimerIfNecessary()
{
    if (m_timer.isActive())
        return;

    if (m_plugins.isEmpty())
        return;

    double nextFireInterval = static_cast<double>(m_pluginAllowedRunTime) - (currentTime() - m_oldestStartTime);
    m_timer.startOneShot(nextFireInterval < 0 ? 0 : nextFireInterval);
}

} // namespace WebCore
