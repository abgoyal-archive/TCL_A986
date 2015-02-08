

#ifndef FramelessScrollViewClient_h
#define FramelessScrollViewClient_h

#include "HostWindow.h"

namespace WebCore {
    class FramelessScrollView;

    class FramelessScrollViewClient : public HostWindow {
    public:
        virtual void popupClosed(FramelessScrollView* popup_view) = 0;
    };

} // namespace WebCore

#endif
