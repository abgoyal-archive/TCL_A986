
#include "config.h"
#include "ContextMenuClientHaiku.h"

#include "ContextMenu.h"
#include "HitTestResult.h"
#include "KURL.h"

#include "NotImplemented.h"


namespace WebCore {

void ContextMenuClientHaiku::contextMenuDestroyed()
{
    notImplemented();
}

PlatformMenuDescription ContextMenuClientHaiku::getCustomMenuFromDefaultItems(ContextMenu* menu)
{
    return menu->platformDescription();
}

void ContextMenuClientHaiku::contextMenuItemSelected(ContextMenuItem*, const ContextMenu*)
{
    notImplemented();
}

void ContextMenuClientHaiku::downloadURL(const KURL& url)
{
    notImplemented();
}

void ContextMenuClientHaiku::lookUpInDictionary(Frame*)
{
    notImplemented();
}

void ContextMenuClientHaiku::speak(const String&)
{
    notImplemented();
}

bool ContextMenuClientHaiku::isSpeaking()
{
    notImplemented();
    return false;
}

void ContextMenuClientHaiku::stopSpeaking()
{
    notImplemented();
}

void ContextMenuClientHaiku::searchWithGoogle(const Frame*)
{
    notImplemented();
}

} // namespace WebCore

