

#ifndef ContextMenuClientHaiku_h
#define ContextMenuClientHaiku_h

#include "ContextMenuClient.h"


namespace WebCore {
    class ContextMenu;

    class ContextMenuClientHaiku : public ContextMenuClient {
    public:
        virtual void contextMenuDestroyed();

        virtual PlatformMenuDescription getCustomMenuFromDefaultItems(ContextMenu*);
        virtual void contextMenuItemSelected(ContextMenuItem*, const ContextMenu*);

        virtual void downloadURL(const KURL& url);
        virtual void searchWithGoogle(const Frame*);
        virtual void lookUpInDictionary(Frame*);
        virtual void speak(const String&);
        virtual bool isSpeaking();
        virtual void stopSpeaking();
    };
} // namespace WebCore

#endif // ContextMenuClientHaiku_h

