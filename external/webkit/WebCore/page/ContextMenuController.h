

#ifndef ContextMenuController_h
#define ContextMenuController_h

#include <wtf/Noncopyable.h>
#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class ContextMenu;
    class ContextMenuClient;
    class ContextMenuItem;
    class ContextMenuProvider;
    class Event;
    class Page;

    class ContextMenuController : public Noncopyable {
    public:
        ContextMenuController(Page*, ContextMenuClient*);
        ~ContextMenuController();

        ContextMenuClient* client() { return m_client; }

        ContextMenu* contextMenu() const { return m_contextMenu.get(); }
        void clearContextMenu();

        void handleContextMenuEvent(Event*);
        void showContextMenu(Event*, PassRefPtr<ContextMenuProvider>);

        void contextMenuItemSelected(ContextMenuItem*);

    private:
        ContextMenu* createContextMenu(Event*);
        void showContextMenu(Event*);

        Page* m_page;
        ContextMenuClient* m_client;
        OwnPtr<ContextMenu> m_contextMenu;
        RefPtr<ContextMenuProvider> m_menuProvider;
    };

}

#endif
