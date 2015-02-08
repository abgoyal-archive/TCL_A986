

#ifndef WindowMessageBroadcaster_h
#define WindowMessageBroadcaster_h

#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/Noncopyable.h>

namespace WebCore {

    class WindowMessageListener;

    class WindowMessageBroadcaster : public Noncopyable {
    public:
        static void addListener(HWND, WindowMessageListener*);
        static void removeListener(HWND, WindowMessageListener*);

    private:
        typedef HashSet<WindowMessageListener*> ListenerSet;

        static LRESULT CALLBACK SubclassedWndProc(HWND, UINT, WPARAM, LPARAM);

        WindowMessageBroadcaster(HWND);
        ~WindowMessageBroadcaster();

        void addListener(WindowMessageListener*);
        void removeListener(WindowMessageListener*);
        const ListenerSet& listeners() const { return m_listeners; }

        void destroy();
        void unsubclassWindow();

        WNDPROC originalWndProc() const { return m_originalWndProc; }

        HWND m_subclassedWindow;
        WNDPROC m_originalWndProc;
        ListenerSet m_listeners;
    };

} // namespace WebCore

#endif // WindowMessageBroadcaster_h
