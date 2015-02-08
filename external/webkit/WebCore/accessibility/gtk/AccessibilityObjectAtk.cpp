

#include "config.h"
#include "AccessibilityObject.h"

#include <glib-object.h>

#if HAVE(ACCESSIBILITY)

namespace WebCore {

bool AccessibilityObject::accessibilityIgnoreAttachment() const
{
    return false;
}

AccessibilityObjectPlatformInclusion AccessibilityObject::accessibilityPlatformIncludesObject() const
{
    AccessibilityObject* parent = parentObject();
    if (!parent)
        return DefaultBehavior;

    if (isMenuListPopup() || isMenuListOption())
        return IgnoreObject;

    // When a list item is made up entirely of children (e.g. paragraphs)
    // the list item gets ignored. We need it.
    if (isGroup() && parent->isList())
        return IncludeObject;

    // Entries and password fields have extraneous children which we want to ignore.
    if (parent->isPasswordField() || parent->isTextControl())
        return IgnoreObject;

    // The object containing the text should implement AtkText itself.
    if (roleValue() == StaticTextRole)
        return IgnoreObject;

    return DefaultBehavior;
}

AccessibilityObjectWrapper* AccessibilityObject::wrapper() const
{
    return m_wrapper;
}

void AccessibilityObject::setWrapper(AccessibilityObjectWrapper* wrapper)
{
    if (wrapper == m_wrapper)
        return;

    if (m_wrapper)
        g_object_unref(m_wrapper);

    m_wrapper = wrapper;

    if (m_wrapper)
        g_object_ref(m_wrapper);
}

} // namespace WebCore

#endif // HAVE(ACCESSIBILITY)
