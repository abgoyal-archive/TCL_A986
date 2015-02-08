

#include "config.h"
#include "PlatformMouseEvent.h"

#include <AEEEvent.h>
#include <AEEPointerHelpers.h>
#include <AEEStdDef.h>
#include <AEEVCodes.h>

namespace WebCore {

PlatformMouseEvent::PlatformMouseEvent(AEEEvent event, uint16 wParam, uint32 dwParam)
{
    switch (event) {
    case EVT_POINTER_DOWN:
        m_eventType = MouseEventPressed;
        break;
    case EVT_POINTER_UP:
        m_eventType = MouseEventReleased;
        break;
    case EVT_POINTER_MOVE:
    case EVT_POINTER_STALE_MOVE:
        m_eventType = MouseEventMoved;
        break;
    default:
        m_eventType = MouseEventMoved;
        break;
    };

    char* dwParamStr = reinterpret_cast<char*>(dwParam);

    int x, y;
    AEE_POINTER_GET_XY(dwParamStr, &x, &y);
    m_position = IntPoint(x, y);
    // Use IDisplay, so position and global position are the same.
    m_globalPosition = m_position;

    uint32 keyModifiers = AEE_POINTER_GET_KEY_MODIFIERS(dwParamStr);
    m_shiftKey = keyModifiers & (KB_LSHIFT | KB_RSHIFT);
    m_ctrlKey  = keyModifiers & (KB_LCTRL | KB_RCTRL);
    m_altKey   = keyModifiers & (KB_LALT | KB_RALT);
    m_metaKey  = m_altKey;

    uint16 mouseModifiers = AEE_POINTER_GET_MOUSE_MODIFIERS(dwParamStr);
    if (mouseModifiers & AEE_POINTER_MOUSE_LBUTTON)
        m_button = LeftButton;
    else if (mouseModifiers & AEE_POINTER_MOUSE_RBUTTON)
        m_button = RightButton;
    else if (mouseModifiers & AEE_POINTER_MOUSE_MBUTTON)
        m_button = MiddleButton;
    else
        m_button = NoButton;

    // AEE_POINTER_GET_TIME returns milliseconds
    m_timestamp = AEE_POINTER_GET_TIME(dwParamStr) * 0.001;

    m_clickCount = AEE_POINTER_GET_CLICKCOUNT(dwParamStr);
}

} // namespace WebCore

