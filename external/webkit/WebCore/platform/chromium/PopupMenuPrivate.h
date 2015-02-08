

#ifndef PopupMenuPrivate_h
#define PopupMenuPrivate_h

#include "RefPtr.h"

namespace WebCore {

    class PopupContainer;

    struct PopupMenuPrivate {
        RefPtr<PopupContainer> popup;
    };

} // namespace WebCore

#endif
