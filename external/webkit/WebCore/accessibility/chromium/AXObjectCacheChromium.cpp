

#include "config.h"
#include "AXObjectCache.h"
#include "AccessibilityObject.h"
#include "Chrome.h"
#include "ChromeClientChromium.h"
#include "FrameView.h"

namespace WebCore {

static ChromeClientChromium* toChromeClientChromium(FrameView* view)
{
    Page* page = view->frame() ? view->frame()->page() : 0;
    if (!page)
        return 0;

    return static_cast<ChromeClientChromium*>(page->chrome()->client());
}

void AXObjectCache::detachWrapper(AccessibilityObject* obj)
{
    // In Chromium, AccessibilityObjects are wrapped lazily.
    if (AccessibilityObjectWrapper* wrapper = obj->wrapper())
        wrapper->detach();
}

void AXObjectCache::attachWrapper(AccessibilityObject*)
{
    // In Chromium, AccessibilityObjects are wrapped lazily.
}

void AXObjectCache::postPlatformNotification(AccessibilityObject* obj, AXNotification notification)
{
    if (notification != AXCheckedStateChanged)
        return;

    if (!obj || !obj->document() || !obj->documentFrameView())
        return;

    ChromeClientChromium* client = toChromeClientChromium(obj->documentFrameView());
    if (client)
        client->didChangeAccessibilityObjectState(obj);
}

void AXObjectCache::handleFocusedUIElementChanged(RenderObject*, RenderObject*)
{
}

void AXObjectCache::handleScrolledToAnchor(const Node*)
{
}

} // namespace WebCore
