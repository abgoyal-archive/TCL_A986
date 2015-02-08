

#ifndef WindowMessageListener_h
#define WindowMessageListener_h

typedef struct HWND__* HWND;
typedef long LPARAM;
typedef unsigned UINT;
typedef unsigned WPARAM;

namespace WebCore {

    class WindowMessageListener {
    public:
        virtual void windowReceivedMessage(HWND, UINT message, WPARAM, LPARAM) = 0;
    };

} // namespace WebCore

#endif // WindowMessageListener_h
