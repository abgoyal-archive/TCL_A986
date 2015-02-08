

#include "HostWindow.h"

namespace WebKitAPITest {

static LPCWSTR hostWindowClassName = L"HostWindow";

HostWindow::HostWindow()
    : m_window(0)
{
}

bool HostWindow::initialize()
{
    registerWindowClass();
    m_window = CreateWindowExW(0, hostWindowClassName, L"WebKitAPITest", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, 0, 0, GetModuleHandle(0), 0);
    return m_window;
}

HostWindow::~HostWindow()
{
    if (!IsWindow(m_window))
        return;
    DestroyWindow(m_window);
}

RECT HostWindow::clientRect() const
{
    RECT rect = {0};
    if (!GetClientRect(m_window, &rect)) {
        RECT emptyRect = {0};
        return emptyRect;
    }
    return rect;
}

void HostWindow::registerWindowClass()
{
    static bool initialized;
    if (initialized)
        return;
    initialized = true;

    WNDCLASSEXW wndClass = {0};
    wndClass.cbSize = sizeof(wndClass);
    wndClass.style = CS_HREDRAW | CS_VREDRAW;
    wndClass.lpfnWndProc = wndProc;
    wndClass.hCursor = LoadCursor(0, IDC_ARROW);
    wndClass.hInstance = GetModuleHandle(0);
    wndClass.lpszClassName = hostWindowClassName;

    RegisterClassExW(&wndClass);
}

LRESULT HostWindow::wndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    return DefWindowProcW(hWnd, uMsg, wParam, lParam);
}

} // namespace WebKitAPITest
