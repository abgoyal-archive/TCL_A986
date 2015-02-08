

#include "config.h"
#include "PlatformWheelEvent.h"

#include "FloatPoint.h"
#include "FloatSize.h"
#include <windows.h>
#include <windowsx.h>

namespace WebCore {

#define HIGH_BIT_MASK_SHORT 0x8000
#define SPI_GETWHEELSCROLLCHARS 0x006C

static IntPoint positionForEvent(HWND hWnd, LPARAM lParam)
{
    POINT point = {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
    ScreenToClient(hWnd, &point);
    return point;
}

static IntPoint globalPositionForEvent(HWND hWnd, LPARAM lParam)
{
    POINT point = {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
    return point;
}

static int horizontalScrollChars()
{
    static ULONG scrollChars;
    if (!scrollChars && !SystemParametersInfo(SPI_GETWHEELSCROLLCHARS, 0, &scrollChars, 0))
        scrollChars = 1;
    return scrollChars;
}

static int verticalScrollLines()
{
    static ULONG scrollLines;
    if (!scrollLines && !SystemParametersInfo(SPI_GETWHEELSCROLLLINES, 0, &scrollLines, 0))
        scrollLines = 3;
    return scrollLines;
}

PlatformWheelEvent::PlatformWheelEvent(HWND hWnd, const FloatSize& delta, const FloatPoint& location)
    : m_isAccepted(false)
    , m_shiftKey(false)
    , m_ctrlKey(false)
    , m_altKey(false)
    , m_metaKey(false)
{
    m_deltaX = delta.width();
    m_deltaY = delta.height();

    m_wheelTicksX = m_deltaX;
    m_wheelTicksY = m_deltaY;

    // Global Position is just x, y location of event
    POINT point = {location.x(), location.y()};
    m_globalPosition = point;

    // Position needs to be translated to our client
    ScreenToClient(hWnd, &point);
    m_position = point;
}

PlatformWheelEvent::PlatformWheelEvent(HWND hWnd, WPARAM wParam, LPARAM lParam, bool isMouseHWheel)
    : m_position(positionForEvent(hWnd, lParam))
    , m_globalPosition(globalPositionForEvent(hWnd, lParam))
    , m_isAccepted(false)
    , m_shiftKey(wParam & MK_SHIFT)
    , m_ctrlKey(wParam & MK_CONTROL)
    , m_altKey(GetKeyState(VK_MENU) & HIGH_BIT_MASK_SHORT)
    , m_metaKey(m_altKey) // FIXME: We'll have to test other browsers
{
    // How many pixels should we scroll per line?  Gecko uses the height of the
    // current line, which means scroll distance changes as you go through the
    // page or go to different pages.  IE 7 is ~50 px/line, although the value
    // seems to vary slightly by page and zoom level.  Since IE 7 has a
    // smoothing algorithm on scrolling, it can get away with slightly larger
    // scroll values without feeling jerky.  Here we use 100 px per three lines
    // (the default scroll amount on Windows is three lines per wheel tick).
    static const float cScrollbarPixelsPerLine = 100.0f / 3.0f;
    float delta = GET_WHEEL_DELTA_WPARAM(wParam) / static_cast<float>(WHEEL_DELTA);
    if (isMouseHWheel) {
        // Windows is <-- -/+ -->, WebKit wants <-- +/- -->, so we negate
        // |delta| after saving the original value on the wheel tick member.
        m_wheelTicksX = delta;
        m_wheelTicksY = 0;
        delta = -delta;
    } else {
        // Even though we use shift + vertical wheel to scroll horizontally in
        // WebKit, we still note it as a vertical scroll on the wheel tick
        // member, so that the DOM event we later construct will match the real
        // hardware event better.
        m_wheelTicksX = 0;
        m_wheelTicksY = delta;
    }
    if (isMouseHWheel || m_shiftKey) {
        m_deltaX = delta * static_cast<float>(horizontalScrollChars()) * cScrollbarPixelsPerLine;
        m_deltaY = 0;
        m_granularity = ScrollByPixelWheelEvent;
    } else {
        m_deltaX = 0;
        m_deltaY = delta;
        int verticalMultiplier = verticalScrollLines();
        m_granularity = (verticalMultiplier == WHEEL_PAGESCROLL) ? ScrollByPageWheelEvent : ScrollByPixelWheelEvent;
        if (m_granularity == ScrollByPixelWheelEvent)
            m_deltaY *= static_cast<float>(verticalMultiplier) * cScrollbarPixelsPerLine;
    }
}

}
