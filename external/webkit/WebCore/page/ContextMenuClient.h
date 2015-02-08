

#ifndef ContextMenuClient_h
#define ContextMenuClient_h

#include "PlatformMenuDescription.h"

namespace WebCore {
    class ContextMenu;
    class ContextMenuItem;
    class Frame;
    class HitTestResult;
    class KURL;
    class String;

    class ContextMenuClient {
    public:
        virtual ~ContextMenuClient() {  }
        virtual void contextMenuDestroyed() = 0;
        
        virtual PlatformMenuDescription getCustomMenuFromDefaultItems(ContextMenu*) = 0;
        virtual void contextMenuItemSelected(ContextMenuItem*, const ContextMenu*) = 0;

        virtual void downloadURL(const KURL& url) = 0;
        virtual void searchWithGoogle(const Frame*) = 0;
        virtual void lookUpInDictionary(Frame*) = 0;
        virtual bool isSpeaking() = 0;
        virtual void speak(const String&) = 0;
        virtual void stopSpeaking() = 0;

#if PLATFORM(MAC)
        virtual void searchWithSpotlight() = 0;
#endif
    };
}

#endif
