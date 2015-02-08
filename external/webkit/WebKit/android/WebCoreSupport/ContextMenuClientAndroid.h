

#ifndef ContextMenuClientAndroid_h
#define ContextMenuClientAndroid_h

#include "ContextMenuClient.h"

namespace WebCore {

class ContextMenuClientAndroid : public ContextMenuClient {
public:
    virtual void contextMenuDestroyed();
    
    virtual PlatformMenuDescription getCustomMenuFromDefaultItems(ContextMenu*);
    virtual void contextMenuItemSelected(ContextMenuItem*, const ContextMenu*);
    
    virtual void downloadURL(const KURL& url);
    virtual void copyImageToClipboard(const HitTestResult&);
    virtual void searchWithGoogle(const Frame*);
    virtual void lookUpInDictionary(Frame*);
    virtual void speak(const String&);
    virtual void stopSpeaking();
    virtual bool isSpeaking();
};

} // namespace WebCore

#endif  // ContextMenuClientAndroid_h
