

#include "config.h"
#include "ContextMenuClientQt.h"

#include "ContextMenu.h"
#include "HitTestResult.h"
#include "KURL.h"
#include "RefCounted.h"
#include "NotImplemented.h"

#include <stdio.h>

namespace WebCore {
    
void ContextMenuClientQt::contextMenuDestroyed()
{
    delete this;
}

PlatformMenuDescription ContextMenuClientQt::getCustomMenuFromDefaultItems(ContextMenu* menu)
{
    // warning: this transfers the ownership to the caller
    return menu->releasePlatformDescription();
}

void ContextMenuClientQt::contextMenuItemSelected(ContextMenuItem*, const ContextMenu*)
{
    notImplemented();
}

void ContextMenuClientQt::downloadURL(const KURL&)
{
    notImplemented();
}

void ContextMenuClientQt::lookUpInDictionary(Frame*)
{
    notImplemented();
}

void ContextMenuClientQt::speak(const String&)
{
    notImplemented();
}

bool ContextMenuClientQt::isSpeaking()
{
    notImplemented();
    return false;
}

void ContextMenuClientQt::stopSpeaking()
{
    notImplemented();
}

void ContextMenuClientQt::searchWithGoogle(const Frame*)
{
    notImplemented();
}

}

