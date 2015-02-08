

#include "config.h"

#if ENABLE(SVG)
#include "RenderSVGTransformableContainer.h"

#include "SVGShadowTreeElements.h"
#include "SVGStyledTransformableElement.h"

namespace WebCore {
    
RenderSVGTransformableContainer::RenderSVGTransformableContainer(SVGStyledTransformableElement* node)
    : RenderSVGContainer(node)
{
}

const AffineTransform& RenderSVGTransformableContainer::localToParentTransform() const
{
    return m_localTransform;
}

AffineTransform RenderSVGTransformableContainer::localTransform() const
{
    return m_localTransform;
}

void RenderSVGTransformableContainer::calculateLocalTransform()
{
    m_localTransform = static_cast<SVGStyledTransformableElement*>(node())->animatedLocalTransform();
    if (!node()->hasTagName(SVGNames::gTag) || !static_cast<SVGGElement*>(node())->isShadowTreeContainerElement())
        return;

    FloatSize translation = static_cast<SVGShadowTreeContainerElement*>(node())->containerTranslation();
    if (translation.width() == 0 && translation.height() == 0)
        return;

    m_localTransform.translate(translation.width(), translation.height());
}

}

#endif // ENABLE(SVG)
