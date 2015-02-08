

#include "config.h"
#include "ContextMenuClientWx.h"
#include "NotImplemented.h"
#include "ContextMenu.h"
#include <stdio.h>

namespace WebCore {

ContextMenuClientWx::~ContextMenuClientWx()
{
}

void ContextMenuClientWx::contextMenuDestroyed()
{
    delete this;
}

PlatformMenuDescription ContextMenuClientWx::getCustomMenuFromDefaultItems(ContextMenu* menu)
{
    return menu->releasePlatformDescription();
}

void ContextMenuClientWx::contextMenuItemSelected(ContextMenuItem*,
                                                   const ContextMenu*)
{
    notImplemented();
}

void ContextMenuClientWx::downloadURL(const KURL&)
{
    notImplemented();
}

void ContextMenuClientWx::copyImageToClipboard(const HitTestResult&)
{
    notImplemented();
}

void ContextMenuClientWx::searchWithGoogle(const Frame*)
{
    notImplemented();
}

void ContextMenuClientWx::lookUpInDictionary(Frame*)
{
    notImplemented();
}

void ContextMenuClientWx::speak(const String&)
{
    notImplemented();
}

void ContextMenuClientWx::stopSpeaking()
{
    notImplemented();
}

bool ContextMenuClientWx::isSpeaking()
{
    notImplemented();
    return false;
}

}
