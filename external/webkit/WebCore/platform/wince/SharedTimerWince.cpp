

#include "config.h"
#include "SharedTimer.h"

#include "Page.h"
#include "SystemTime.h"
#include "Widget.h"
#include <wtf/Assertions.h>
#include <wtf/CurrentTime.h>
#include <windows.h>

namespace JSC {
extern void* g_stackBase;
}

namespace WebCore {

enum {
    TimerIdNone = 0,
    TimerIdAuto,
    TimerIdManual,
};
static UINT timerID = TimerIdNone;

static void (*sharedTimerFiredFunction)();

static HWND timerWindowHandle = 0;
const LPCWSTR kTimerWindowClassName = L"TimerWindowClass";

LRESULT CALLBACK TimerWindowWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    int dummy;
    JSC::g_stackBase = &dummy;

    if (message == WM_TIMER) {
        if (timerID != TimerIdNone)
            sharedTimerFiredFunction();
    } else if (message == WM_USER)    {
        if (timerID = TimerIdManual) {
            sharedTimerFiredFunction();
            PostMessage(hWnd, WM_USER, 0, 0);
        }
    } else {
        JSC::g_stackBase = 0;
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    JSC::g_stackBase = 0;
    return 0;
}

static void initializeOffScreenTimerWindow()
{
    if (timerWindowHandle)
        return;

    WNDCLASS wcex = {0};
    wcex.lpfnWndProc    = TimerWindowWndProc;
    wcex.hInstance      = Page::instanceHandle();
    wcex.lpszClassName  = kTimerWindowClassName;
    RegisterClass(&wcex);

    timerWindowHandle = CreateWindow(kTimerWindowClassName, 0, 0,
       CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, 0, 0, Page::instanceHandle(), 0);
}

void setSharedTimerFiredFunction(void (*f)())
{
    sharedTimerFiredFunction = f;
}

#define USER_TIMER_MAXIMUM  0x7FFFFFFF
#define USER_TIMER_MINIMUM  0x0000000A

void setSharedTimerFireTime(double fireTime)
{
    ASSERT(sharedTimerFiredFunction);

    double interval = (fireTime - currentTime()) * 1000.;
    unsigned intervalInMS = interval < USER_TIMER_MINIMUM
        ? USER_TIMER_MINIMUM
        : interval > USER_TIMER_MAXIMUM
        ? USER_TIMER_MAXIMUM
        : static_cast<unsigned>(interval);

    if (timerID == TimerIdAuto) {
        KillTimer(timerWindowHandle, TimerIdAuto);
        timerID = TimerIdNone;
    }

    initializeOffScreenTimerWindow();
    if (SetTimer(timerWindowHandle, TimerIdAuto, intervalInMS, 0))
        timerID = TimerIdAuto;
    else if (timerID != TimerIdManual)
        PostMessage(timerWindowHandle, WM_USER, 0, 0);
}

void stopSharedTimer()
{
    if (timerID == TimerIdAuto)
        KillTimer(timerWindowHandle, TimerIdAuto);

    timerID = TimerIdNone;
}

}
