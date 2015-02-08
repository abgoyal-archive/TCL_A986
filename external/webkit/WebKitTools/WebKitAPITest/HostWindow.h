

#ifndef HostWindow_h
#define HostWindow_h

#include <windows.h>
#include <wtf/Noncopyable.h>

namespace WebKitAPITest {

class HostWindow : public Noncopyable {
public:
    HostWindow();
    ~HostWindow();
    bool initialize();

    RECT clientRect() const;
    HWND window() const { return m_window; }

private:
    static void registerWindowClass();
    static LRESULT CALLBACK wndProc(HWND, UINT uMsg, WPARAM, LPARAM);

    HWND m_window;
};

} // namespace WebKitAPITest

#endif // HostWindow_h
