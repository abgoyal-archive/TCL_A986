

#ifndef PluginMessageThrottlerWin_h
#define PluginMessageThrottlerWin_h

#include "Timer.h"

#include <windows.h>

namespace WebCore {
    class PluginView;

    struct PluginMessage {
        HWND hWnd;
        UINT msg;
        WPARAM wParam;
        LPARAM lParam;

        struct PluginMessage* next;
    };

    class PluginMessageThrottlerWin {
    public:
        PluginMessageThrottlerWin(PluginView*);
        ~PluginMessageThrottlerWin();

        void appendMessage(HWND, UINT msg, WPARAM, LPARAM);

    private:
        void messageThrottleTimerFired(Timer<PluginMessageThrottlerWin>*);
        PluginMessage* allocateMessage();
        bool isInlineMessage(PluginMessage* message);
        void freeMessage(PluginMessage* message);

        PluginView* m_pluginView;
        PluginMessage* m_back;
        PluginMessage* m_front;

        static const int NumInlineMessages = 4;
        PluginMessage m_inlineMessages[NumInlineMessages];
        PluginMessage* m_freeInlineMessages;

        Timer<PluginMessageThrottlerWin> m_messageThrottleTimer;
    };

} // namespace WebCore

#endif // PluginMessageThrottlerWin_h
