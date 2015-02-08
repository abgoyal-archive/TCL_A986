

#include "config.h"
#include "AccessibilityObject.h"

#if HAVE(ACCESSIBILITY)

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

#endif // HAVE(ACCESSIBILITY)
