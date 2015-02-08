

#ifndef SVGShadowTreeElements_h
#define SVGShadowTreeElements_h

#if ENABLE(SVG)
#include "SVGGElement.h"
#include "SVGLength.h"

namespace WebCore {

class FloatSize;

class SVGShadowTreeContainerElement : public SVGGElement {
public:
    SVGShadowTreeContainerElement(Document*);
    virtual ~SVGShadowTreeContainerElement();

    virtual bool isShadowTreeContainerElement() const { return true; }

    FloatSize containerTranslation() const;
    void setContainerOffset(const SVGLength& x, const SVGLength& y)
    {
        m_xOffset = x;
        m_yOffset = y;
    }

private:
    SVGLength m_xOffset;
    SVGLength m_yOffset;
};

class SVGShadowTreeRootElement : public SVGShadowTreeContainerElement {
public:
    SVGShadowTreeRootElement(Document*, Node* shadowParent);
    virtual ~SVGShadowTreeRootElement();

    virtual bool isShadowNode() const { return m_shadowParent; }
    virtual Node* shadowParentNode() { return m_shadowParent; }

    void attachElement(PassRefPtr<RenderStyle>, RenderArena*);

private:
    Node* m_shadowParent;
};

}

#endif
#endif
