

#ifndef RenderSVGShadowTreeRootContainer_h
#define RenderSVGShadowTreeRootContainer_h

#if ENABLE(SVG)
#include "RenderSVGTransformableContainer.h"

namespace WebCore {
    
class SVGUseElement;
class SVGShadowTreeRootElement;

class RenderSVGShadowTreeRootContainer : public RenderSVGTransformableContainer {
public:
    RenderSVGShadowTreeRootContainer(SVGUseElement*);
    virtual ~RenderSVGShadowTreeRootContainer();

    void markShadowTreeForRecreation() { m_recreateTree = true; }
    void updateStyle(Node::StyleChange);
    virtual void updateFromElement();

private:
    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    bool m_recreateTree;
    RefPtr<SVGShadowTreeRootElement> m_shadowRoot;
};

}

#endif
#endif
