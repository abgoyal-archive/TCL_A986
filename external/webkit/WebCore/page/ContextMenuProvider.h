

#ifndef ContextMenuProvider_h
#define ContextMenuProvider_h

#include <wtf/RefCounted.h>

namespace WebCore {

class ContextMenu;
class ContextMenuItem;

class ContextMenuProvider : public RefCounted<ContextMenuProvider> {
public:
    virtual ~ContextMenuProvider() { };

    virtual void populateContextMenu(ContextMenu*) = 0;
    virtual void contextMenuItemSelected(ContextMenuItem*) = 0;
    virtual void contextMenuCleared() = 0;
};

}

#endif // ContextMenuProvider_h
