

#ifndef ContextMenuClientQt_h
#define ContextMenuClientQt_h

#include "ContextMenuClient.h"

#include <RefCounted.h>

namespace WebCore {
    class ContextMenu;

    class ContextMenuClientQt : public ContextMenuClient
    {
    public:
        virtual void contextMenuDestroyed();

        virtual PlatformMenuDescription getCustomMenuFromDefaultItems(ContextMenu*);
        virtual void contextMenuItemSelected(ContextMenuItem*, const ContextMenu*);

        virtual void downloadURL(const KURL& url);
        virtual void lookUpInDictionary(Frame*);
        virtual void speak(const String&);
        virtual bool isSpeaking();
        virtual void stopSpeaking();
        virtual void searchWithGoogle(const Frame*);
    };
}

#endif
