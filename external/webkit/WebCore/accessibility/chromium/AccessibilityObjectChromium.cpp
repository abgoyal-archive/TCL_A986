

#include "config.h"
#include "AccessibilityObject.h"

namespace WebCore {

bool AccessibilityObject::accessibilityIgnoreAttachment() const
{
    return false;
}

AccessibilityObjectPlatformInclusion AccessibilityObject::accessibilityPlatformIncludesObject() const
{
    if (isMenuListPopup() || isMenuListOption())
        return IgnoreObject;

    return DefaultBehavior;
}

} // namespace WebCore
