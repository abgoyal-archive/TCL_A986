

#include "config.h"
#include "InspectorClientHaiku.h"

#include "PlatformString.h"

#include "NotImplemented.h"


namespace WebCore {

void InspectorClientHaiku::inspectorDestroyed()
{
    notImplemented();
}

Page* InspectorClientHaiku::createPage()
{
    notImplemented();
    return 0;
}

String InspectorClientHaiku::localizedStringsURL()
{
    notImplemented();
    return String();
}

String InspectorClientHaiku::hiddenPanels()
{
    notImplemented();
    return String();
}

void InspectorClientHaiku::showWindow()
{
    notImplemented();
}

void InspectorClientHaiku::closeWindow()
{
    notImplemented();
}

void InspectorClientHaiku::attachWindow()
{
    notImplemented();
}

void InspectorClientHaiku::detachWindow()
{
    notImplemented();
}

void InspectorClientHaiku::setAttachedWindowHeight(unsigned height)
{
    notImplemented();
}

void InspectorClientHaiku::highlight(Node* node)
{
    notImplemented();
}

void InspectorClientHaiku::hideHighlight()
{
    notImplemented();
}

void InspectorClientHaiku::inspectedURLChanged(const String&)
{
    notImplemented();
}

void InspectorClientHaiku::inspectorWindowObjectCleared()
{
    notImplemented();
}

void InspectorClientHaiku::populateSetting(const String& key, String* value)
{
    notImplemented();
}

void InspectorClientHaiku::storeSetting(const String& key, const String& value)
{
    notImplemented();
}

} // namespace WebCore

