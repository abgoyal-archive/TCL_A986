

#include "config.h"
#include "AccessibleImage.h"

#include <WebCore/AccessibilityRenderObject.h>
#include <WebCore/HTMLNames.h>

using namespace WebCore;
using namespace WebCore::HTMLNames;

AccessibleImage::AccessibleImage(AccessibilityObject* obj)
    : AccessibleBase(obj)
{
    ASSERT_ARG(obj, obj->isImage());
    ASSERT_ARG(obj, obj->isAccessibilityRenderObject());
}

String AccessibleImage::name() const
{
    if (!m_object->isAccessibilityRenderObject())
        return AccessibleBase::name();

    AccessibilityRenderObject* obj = static_cast<AccessibilityRenderObject*>(m_object);

    String ariaLabel = obj->ariaLabeledByAttribute();
    if (!ariaLabel.isEmpty())
        return ariaLabel;

    const AtomicString& altText = obj->getAttribute(HTMLNames::altAttr);
    if (!altText.isEmpty())
        return altText;

    return AccessibleBase::name();
}
