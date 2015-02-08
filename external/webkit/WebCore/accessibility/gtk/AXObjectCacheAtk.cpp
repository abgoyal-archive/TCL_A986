

#include "config.h"
#include "AXObjectCache.h"

#include "AccessibilityObject.h"
#include "AccessibilityObjectWrapperAtk.h"

namespace WebCore {

void AXObjectCache::detachWrapper(AccessibilityObject* obj)
{
    webkit_accessible_detach(WEBKIT_ACCESSIBLE(obj->wrapper()));
}

void AXObjectCache::attachWrapper(AccessibilityObject* obj)
{
    AtkObject* atkObj = ATK_OBJECT(webkit_accessible_new(obj));
    obj->setWrapper(atkObj);
    g_object_unref(atkObj);
}

void AXObjectCache::postPlatformNotification(AccessibilityObject* coreObject, AXNotification notification)
{
    if (notification == AXCheckedStateChanged) {
        if (!coreObject->isCheckboxOrRadio())
            return;
        g_signal_emit_by_name(coreObject->wrapper(), "state-change", "checked", coreObject->isChecked());
    } else if (notification == AXSelectedChildrenChanged) {
        if (!coreObject->isListBox())
            return;
        g_signal_emit_by_name(coreObject->wrapper(), "selection-changed");
    }
}

void AXObjectCache::handleFocusedUIElementChanged(RenderObject* oldFocusedRender, RenderObject* newFocusedRender)
{
    RefPtr<AccessibilityObject> oldObject = getOrCreate(oldFocusedRender);
    if (oldObject) {
        g_signal_emit_by_name(oldObject->wrapper(), "focus-event", false);
        g_signal_emit_by_name(oldObject->wrapper(), "state-change", "focused", false);
    }
    RefPtr<AccessibilityObject> newObject = getOrCreate(newFocusedRender);
    if (newObject) {
        g_signal_emit_by_name(newObject->wrapper(), "focus-event", true);
        g_signal_emit_by_name(newObject->wrapper(), "state-change", "focused", true);
    }
}

void AXObjectCache::handleScrolledToAnchor(const Node*)
{
}

} // namespace WebCore
