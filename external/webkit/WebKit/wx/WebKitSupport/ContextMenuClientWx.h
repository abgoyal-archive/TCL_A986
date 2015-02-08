

#ifndef ContextMenuClientWx_h
#define ContextMenuClientWx_h

#include "ContextMenuClient.h"
#include "PlatformMenuDescription.h"

namespace WebCore {

class ContextMenuClientWx : public ContextMenuClient {
public:
    virtual ~ContextMenuClientWx();
    virtual void contextMenuDestroyed();

    virtual PlatformMenuDescription getCustomMenuFromDefaultItems(ContextMenu*);
    virtual void contextMenuItemSelected(ContextMenuItem*,
                                         const ContextMenu*);

    virtual void downloadURL(const KURL&);
    virtual void copyImageToClipboard(const HitTestResult&);
    virtual void searchWithGoogle(const Frame*);
    virtual void lookUpInDictionary(Frame*);
    virtual void speak(const String&);
    virtual void stopSpeaking();
    virtual bool isSpeaking();
};

}

#endif // ContextMenuClientWx_h
