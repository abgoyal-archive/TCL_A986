

#include "config.h"
#include "WindowMessageBroadcaster.h"

#include "WindowMessageListener.h"

namespace WebCore {

typedef HashMap<HWND, WindowMessageBroadcaster*> InstanceMap;

static InstanceMap& instancesMap()
{
    static InstanceMap instances;
    return instances;
}

void WindowMessageBroadcaster::addListener(HWND hwnd, WindowMessageListener* listener)
{
    WindowMessageBroadcaster* broadcaster = instancesMap().get(hwnd);
    if (!broadcaster) {
        broadcaster = new WindowMessageBroadcaster(hwnd);
        instancesMap().add(hwnd, broadcaster);
    }

    broadcaster->addListener(listener);
}

void WindowMessageBroadcaster::removeListener(HWND hwnd, WindowMessageListener* listener)
{
    WindowMessageBroadcaster* broadcaster = instancesMap().get(hwnd);
    if (!broadcaster)
        return;

    broadcaster->removeListener(listener);
}

WindowMessageBroadcaster::WindowMessageBroadcaster(HWND hwnd)
    : m_subclassedWindow(hwnd)
    , m_originalWndProc(0)
{
    ASSERT_ARG(hwnd, IsWindow(hwnd));
}

WindowMessageBroadcaster::~WindowMessageBroadcaster()
{
}

void WindowMessageBroadcaster::addListener(WindowMessageListener* listener)
{
    if (m_listeners.isEmpty()) {
        ASSERT(!m_originalWndProc);
#pragma warning(disable: 4244 4312)
        m_originalWndProc = reinterpret_cast<WNDPROC>(SetWindowLongPtr(m_subclassedWindow, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(SubclassedWndProc)));
    }

    m_listeners.add(listener);
}

void WindowMessageBroadcaster::removeListener(WindowMessageListener* listener)
{
    ListenerSet::iterator found = m_listeners.find(listener);
    if (found == m_listeners.end())
        return;

    m_listeners.remove(found);

    if (m_listeners.isEmpty())
        destroy();
}

void WindowMessageBroadcaster::destroy()
{
    m_listeners.clear();
    unsubclassWindow();
    instancesMap().remove(m_subclassedWindow);
    delete this;
}

void WindowMessageBroadcaster::unsubclassWindow()
{
    SetWindowLongPtr(m_subclassedWindow, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(m_originalWndProc));
    m_originalWndProc = 0;
}

LRESULT CALLBACK WindowMessageBroadcaster::SubclassedWndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    WindowMessageBroadcaster* broadcaster = instancesMap().get(hwnd);
    ASSERT(broadcaster);

    ListenerSet::const_iterator end = broadcaster->listeners().end();
    for (ListenerSet::const_iterator it = broadcaster->listeners().begin(); it != end; ++it)
        (*it)->windowReceivedMessage(hwnd, message, wParam, lParam);

    WNDPROC originalWndProc = broadcaster->originalWndProc();

    // This will delete broadcaster.
    if (message == WM_DESTROY)
        broadcaster->destroy();

    return CallWindowProc(originalWndProc, hwnd, message, wParam, lParam);
}

} // namespace WebCore
