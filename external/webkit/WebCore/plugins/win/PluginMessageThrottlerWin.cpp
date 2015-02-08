

#include "config.h"
#include "PluginMessageThrottlerWin.h"

#include "PluginView.h"
#include <wtf/ASCIICType.h>

using namespace WTF;

namespace WebCore {

static const double MessageThrottleTimeInterval = 0.001;

PluginMessageThrottlerWin::PluginMessageThrottlerWin(PluginView* pluginView)
    : m_back(0), m_front(0)
    , m_pluginView(pluginView)
    , m_messageThrottleTimer(this, &PluginMessageThrottlerWin::messageThrottleTimerFired)
{
    // Initialize the free list with our inline messages
    for (unsigned i = 0; i < NumInlineMessages - 1; i++)
        m_inlineMessages[i].next = &m_inlineMessages[i + 1];
    m_inlineMessages[NumInlineMessages - 1].next = 0;
    m_freeInlineMessages = &m_inlineMessages[0];
}

PluginMessageThrottlerWin::~PluginMessageThrottlerWin()
{
    PluginMessage* next;

    for (PluginMessage* message = m_front; message; message = next) {
        next = message->next;
        freeMessage(message);
    }
}

void PluginMessageThrottlerWin::appendMessage(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    PluginMessage* message = allocateMessage();

    message->hWnd = hWnd;
    message->msg = msg;
    message->wParam = wParam;
    message->lParam = lParam;
    message->next = 0;

    if (m_back)
        m_back->next = message;
    m_back = message;
    if (!m_front)
        m_front = message;

    if (!m_messageThrottleTimer.isActive())
        m_messageThrottleTimer.startOneShot(MessageThrottleTimeInterval);
}

void PluginMessageThrottlerWin::messageThrottleTimerFired(Timer<PluginMessageThrottlerWin>*)
{
    PluginMessage* message = m_front;
    m_front = m_front->next;
    if (message == m_back)
        m_back = 0;

    // Protect the PluginView from destruction while calling its window proc.
    // <rdar://problem/6930280>
    RefPtr<PluginView> protect(m_pluginView);
    ::CallWindowProc(m_pluginView->pluginWndProc(), message->hWnd, message->msg, message->wParam, message->lParam);

    freeMessage(message);

    if (m_front)
        m_messageThrottleTimer.startOneShot(MessageThrottleTimeInterval);
}

PluginMessage* PluginMessageThrottlerWin::allocateMessage()
{
    PluginMessage *message;

    if (m_freeInlineMessages) {
        message = m_freeInlineMessages;
        m_freeInlineMessages = message->next;
    } else
        message = new PluginMessage;

    return message;
}

bool PluginMessageThrottlerWin::isInlineMessage(PluginMessage* message)
{
    return message >= &m_inlineMessages[0] && message <= &m_inlineMessages[NumInlineMessages - 1];
}

void PluginMessageThrottlerWin::freeMessage(PluginMessage* message)
{
    if (isInlineMessage(message)) {
        message->next = m_freeInlineMessages;
        m_freeInlineMessages = message;
    } else
        delete message;
}

} // namespace WebCore
