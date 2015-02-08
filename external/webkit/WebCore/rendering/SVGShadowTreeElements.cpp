

#include "config.h"

#if ENABLE(SVG)
#include "SVGShadowTreeElements.h"

#include "Document.h"
#include "FloatSize.h"
#include "RenderObject.h"
#include "SVGNames.h"

namespace WebCore {

// SVGShadowTreeContainerElement
SVGShadowTreeContainerElement::SVGShadowTreeContainerElement(Document* document)
    : SVGGElement(SVGNames::gTag, document)
{
}
    
SVGShadowTreeContainerElement::~SVGShadowTreeContainerElement()
{
}

FloatSize SVGShadowTreeContainerElement::containerTranslation() const
{
    return FloatSize(m_xOffset.value(this), m_yOffset.value(this));
}

// SVGShadowTreeRootElement
SVGShadowTreeRootElement::SVGShadowTreeRootElement(Document* document, Node* shadowParent)
    : SVGShadowTreeContainerElement(document)
    , m_shadowParent(shadowParent)
{
    setInDocument(true);
}

SVGShadowTreeRootElement::~SVGShadowTreeRootElement()
{
}

void SVGShadowTreeRootElement::attachElement(PassRefPtr<RenderStyle> style, RenderArena* arena)
{
    ASSERT(m_shadowParent);

    // Create the renderer with the specified style
    RenderObject* renderer = createRenderer(arena, style.get());
    if (renderer) {
        setRenderer(renderer);
        renderer->setStyle(style);
    }

    // Set these explicitly since this normally happens during an attach()
    setAttached();

    // Add the renderer to the render tree
    if (renderer)
        m_shadowParent->renderer()->addChild(renderer);
}

}

#endif
