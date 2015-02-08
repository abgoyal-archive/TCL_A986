

#ifndef ContextMenu_h
#define ContextMenu_h

#include <wtf/Noncopyable.h>

#include "ContextMenuItem.h"
#include "HitTestResult.h"
#include "PlatformMenuDescription.h"
#include "PlatformString.h"
#if PLATFORM(MAC)
#include <wtf/RetainPtr.h>
#elif PLATFORM(QT)
#include <QMenu>
#endif

namespace WebCore {

    class ContextMenuController;

    class ContextMenu : public Noncopyable
    {
    public:
        ContextMenu(const HitTestResult&);
        ContextMenu(const HitTestResult&, const PlatformMenuDescription);
        ~ContextMenu();

        void populate();
        void addInspectElementItem();
        void checkOrEnableIfNeeded(ContextMenuItem&) const;

        void insertItem(unsigned position, ContextMenuItem&);
        void appendItem(ContextMenuItem&);
        
        ContextMenuItem* itemWithAction(unsigned);
        ContextMenuItem* itemAtIndex(unsigned, const PlatformMenuDescription);

        unsigned itemCount() const;

        HitTestResult hitTestResult() const { return m_hitTestResult; }
        ContextMenuController* controller() const;

        PlatformMenuDescription platformDescription() const;
        void setPlatformDescription(PlatformMenuDescription);

        PlatformMenuDescription releasePlatformDescription();
#if PLATFORM(WX)
        static ContextMenuItem* itemWithId(int);
#endif
    private:
        HitTestResult m_hitTestResult;
#if PLATFORM(MAC)
        // Keep this in sync with the PlatformMenuDescription typedef
        RetainPtr<NSMutableArray> m_platformDescription;
#elif PLATFORM(QT)
        QList<ContextMenuItem> m_items;
#elif PLATFORM(CHROMIUM)
        Vector<ContextMenuItem> m_items;
#else
        PlatformMenuDescription m_platformDescription;
#endif
    };

}

#endif // ContextMenu_h
