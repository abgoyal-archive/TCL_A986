

#include "config.h"
#include "ContextMenuClientAndroid.h"

#include "NotImplemented.h"
#include <wtf/Assertions.h>

namespace WebCore {
    
void ContextMenuClientAndroid::contextMenuDestroyed() { delete this; }

PlatformMenuDescription ContextMenuClientAndroid::getCustomMenuFromDefaultItems(ContextMenu*) { notImplemented(); return 0; }
void ContextMenuClientAndroid::contextMenuItemSelected(ContextMenuItem*, const ContextMenu*) { notImplemented(); }

void ContextMenuClientAndroid::downloadURL(const KURL& url) { notImplemented(); }
void ContextMenuClientAndroid::copyImageToClipboard(const HitTestResult&) { notImplemented(); }
void ContextMenuClientAndroid::searchWithGoogle(const Frame*) { notImplemented(); }
void ContextMenuClientAndroid::lookUpInDictionary(Frame*) { notImplemented(); }
void ContextMenuClientAndroid::speak(const String&) { notImplemented(); }
void ContextMenuClientAndroid::stopSpeaking() { notImplemented(); }
bool ContextMenuClientAndroid::isSpeaking()
{
    notImplemented();
    return false;
}

}
