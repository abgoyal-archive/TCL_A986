

#include "config.h"
#include "MainThread.h"

#include "Assertions.h"
#include "Threading.h"
#if !OS(WINCE)
#include <windows.h>
#endif

namespace WTF {

static HWND threadingWindowHandle;
static UINT threadingFiredMessage;
const LPCWSTR kThreadingWindowClassName = L"ThreadingWindowClass";

LRESULT CALLBACK ThreadingWindowWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    if (message == threadingFiredMessage)
        dispatchFunctionsFromMainThread();
    else
        return DefWindowProc(hWnd, message, wParam, lParam);
    return 0;
}

void initializeMainThreadPlatform()
{
    if (threadingWindowHandle)
        return;

    HWND hWndParent = 0;
#if OS(WINCE)
    WNDCLASS wcex;
    memset(&wcex, 0, sizeof(WNDCLASS));
#else
    WNDCLASSEX wcex;
    memset(&wcex, 0, sizeof(WNDCLASSEX));
    wcex.cbSize = sizeof(WNDCLASSEX);
#endif
    wcex.lpfnWndProc    = ThreadingWindowWndProc;
    wcex.lpszClassName  = kThreadingWindowClassName;
#if OS(WINCE)
    RegisterClass(&wcex);
#else
    RegisterClassEx(&wcex);
    hWndParent = HWND_MESSAGE;
#endif

    threadingWindowHandle = CreateWindow(kThreadingWindowClassName, 0, 0,
       CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, hWndParent, 0, 0, 0);
    threadingFiredMessage = RegisterWindowMessage(L"com.apple.WebKit.MainThreadFired");
}

void scheduleDispatchFunctionsOnMainThread()
{
    ASSERT(threadingWindowHandle);
    PostMessage(threadingWindowHandle, threadingFiredMessage, 0, 0);
}

} // namespace WTF
