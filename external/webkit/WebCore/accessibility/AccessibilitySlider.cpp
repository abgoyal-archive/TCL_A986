

#include "config.h"
#include "AccessibilitySlider.h"

#include "AXObjectCache.h"
#include "HTMLInputElement.h"
#include "HTMLNames.h"
#include "RenderObject.h"
#include "RenderSlider.h"

namespace WebCore {
    
using namespace HTMLNames;

AccessibilitySlider::AccessibilitySlider(RenderObject* renderer)
    : AccessibilityRenderObject(renderer)
{
}

PassRefPtr<AccessibilitySlider> AccessibilitySlider::create(RenderObject* renderer)
{
    return adoptRef(new AccessibilitySlider(renderer));
}

const AccessibilityObject::AccessibilityChildrenVector& AccessibilitySlider::children()
{
    if (!m_haveChildren)
        addChildren();
    return m_children;
}

AccessibilityOrientation AccessibilitySlider::orientation() const
{
    // Default to horizontal in the unknown case.
    if (!m_renderer)
        return AccessibilityOrientationHorizontal;
    
    RenderStyle* style = m_renderer->style();
    if (!style)
        return AccessibilityOrientationHorizontal;
    
    ControlPart styleAppearance = style->appearance();
    switch (styleAppearance) {
    case SliderThumbHorizontalPart:
    case SliderHorizontalPart:
    case MediaSliderPart:
        return AccessibilityOrientationHorizontal;
    
    case SliderThumbVerticalPart: 
    case SliderVerticalPart:
    case MediaVolumeSliderPart:
        return AccessibilityOrientationVertical;
        
    default:
        return AccessibilityOrientationHorizontal;
    }
}
    
void AccessibilitySlider::addChildren()
{
    ASSERT(!m_haveChildren); 
    
    m_haveChildren = true;

    AccessibilitySliderThumb* thumb = static_cast<AccessibilitySliderThumb*>(m_renderer->document()->axObjectCache()->getOrCreate(SliderThumbRole));
    thumb->setParentObject(this);
    m_children.append(thumb);
}

const AtomicString& AccessibilitySlider::getAttribute(const QualifiedName& attribute) const
{
    return element()->getAttribute(attribute);
}

float AccessibilitySlider::valueForRange() const
{
    return element()->value().toFloat();
}

float AccessibilitySlider::maxValueForRange() const
{
    return getAttribute(maxAttr).toFloat();
}

float AccessibilitySlider::minValueForRange() const
{
    return getAttribute(minAttr).toFloat();
}

void AccessibilitySlider::setValue(const String& value)
{
    HTMLInputElement* input = element();
    
    if (input->value() == value)
        return;

    input->setValue(value);

    // Fire change event manually, as RenderSlider::setValueForPosition does.
    input->dispatchFormControlChangeEvent();
}

HTMLInputElement* AccessibilitySlider::element() const
{
    return static_cast<HTMLInputElement*>(m_renderer->node());
}


AccessibilitySliderThumb::AccessibilitySliderThumb()
    : m_parentSlider(0)
{
}

PassRefPtr<AccessibilitySliderThumb> AccessibilitySliderThumb::create()
{
    return adoptRef(new AccessibilitySliderThumb());
}
    
IntRect AccessibilitySliderThumb::elementRect() const
{
    if (!m_parentSlider->renderer())
        return IntRect();

    IntRect intRect = toRenderSlider(m_parentSlider->renderer())->thumbRect();
    FloatQuad floatQuad = m_parentSlider->renderer()->localToAbsoluteQuad(FloatRect(intRect));

    return floatQuad.enclosingBoundingBox();
}

IntSize AccessibilitySliderThumb::size() const
{
    return elementRect().size();
}

} // namespace WebCore
