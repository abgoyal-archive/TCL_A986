


#include "config.h"
#include "AXObjectCache.h"

#include "AccessibilityObject.h"
#include "Chrome.h"
#include "Document.h"
#include "Page.h"
#include "RenderObject.h"

using namespace std;

namespace WebCore {

void AXObjectCache::detachWrapper(AccessibilityObject* obj)
{
    // On Windows, AccessibilityObjects are created when get_accChildCount is
    // called, but they are not wrapped until get_accChild is called, so this
    // object may not have a wrapper.
    if (AccessibilityObjectWrapper* wrapper = obj->wrapper())
        wrapper->detach();
}

void AXObjectCache::attachWrapper(AccessibilityObject*)
{
    // On Windows, AccessibilityObjects are wrapped when the accessibility
    // software requests them via get_accChild.
}

void AXObjectCache::handleScrolledToAnchor(const Node* anchorNode)
{
    // The anchor node may not be accessible. Post the notification for the
    // first accessible object.
    postPlatformNotification(AccessibilityObject::firstAccessibleObjectFromNode(anchorNode), AXScrolledToAnchor);
}

void AXObjectCache::postPlatformNotification(AccessibilityObject* obj, AXNotification notification)
{
    if (!obj)
        return;

    Document* document = obj->document();
    if (!document)
        return;

    Page* page = document->page();
    if (!page || !page->chrome()->platformPageClient())
        return;

    DWORD msaaEvent;
    switch (notification) {
        case AXFocusedUIElementChanged:
        case AXActiveDescendantChanged:
            msaaEvent = EVENT_OBJECT_FOCUS;
            break;

        case AXScrolledToAnchor:
            msaaEvent = EVENT_SYSTEM_SCROLLINGSTART;
            break;

        case AXValueChanged:
        case AXMenuListValueChanged:
            msaaEvent = EVENT_OBJECT_VALUECHANGE;
            break;

        default:
            return;
    }

    // Windows will end up calling get_accChild() on the root accessible
    // object for the WebView, passing the child ID that we specify below. We
    // negate the AXID so we know that the caller is passing the ID of an
    // element, not the index of a child element.

    ASSERT(obj->axObjectID() >= 1);
    ASSERT(obj->axObjectID() <= numeric_limits<LONG>::max());

    NotifyWinEvent(msaaEvent, page->chrome()->platformPageClient(), OBJID_CLIENT, -static_cast<LONG>(obj->axObjectID()));
}

AXID AXObjectCache::platformGenerateAXID() const
{
    static AXID lastUsedID = 0;

    // Generate a new ID. Windows accessibility relies on a positive AXID,
    // ranging from 1 to LONG_MAX.
    AXID objID = lastUsedID;
    do {
        ++objID;
        objID %= std::numeric_limits<LONG>::max();
    } while (objID == 0 || HashTraits<AXID>::isDeletedValue(objID) || m_idsInUse.contains(objID));

    ASSERT(objID >= 1 && objID <= std::numeric_limits<LONG>::max());

    lastUsedID = objID;

    return objID;
}

void AXObjectCache::handleFocusedUIElementChanged(RenderObject*, RenderObject* newFocusedRenderer)
{
    if (!newFocusedRenderer)
        return;

    Page* page = newFocusedRenderer->document()->page();
    if (!page || !page->chrome()->platformPageClient())
        return;

    AccessibilityObject* focusedObject = focusedUIElementForPage(page);
    if (!focusedObject)
        return;

    ASSERT(!focusedObject->accessibilityIsIgnored());

    postPlatformNotification(focusedObject, AXFocusedUIElementChanged);
}

} // namespace WebCore
