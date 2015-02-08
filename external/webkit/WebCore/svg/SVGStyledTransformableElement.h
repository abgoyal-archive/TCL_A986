

#ifndef SVGStyledTransformableElement_h
#define SVGStyledTransformableElement_h

#if ENABLE(SVG)
#include "Path.h"
#include "SVGStyledLocatableElement.h"
#include "SVGTransformable.h"

namespace WebCore {

class AffineTransform;

class SVGStyledTransformableElement : public SVGStyledLocatableElement,
                                      public SVGTransformable {
public:
    SVGStyledTransformableElement(const QualifiedName&, Document*);
    virtual ~SVGStyledTransformableElement();
    
    virtual bool isStyledTransformable() const { return true; }

    virtual AffineTransform getCTM() const;
    virtual AffineTransform getScreenCTM() const;
    virtual SVGElement* nearestViewportElement() const;
    virtual SVGElement* farthestViewportElement() const;
    
    virtual AffineTransform animatedLocalTransform() const;
    virtual AffineTransform* supplementalTransform();

    virtual FloatRect getBBox() const;

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void synchronizeProperty(const QualifiedName&);
    bool isKnownAttribute(const QualifiedName&);

    // "base class" methods for all the elements which render as paths
    virtual Path toPathData() const { return Path(); }
    virtual Path toClipPath() const;
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

protected:
    DECLARE_ANIMATED_PROPERTY(SVGStyledTransformableElement, SVGNames::transformAttr, SVGTransformList*, Transform, transform)

private:
    // Used by <animateMotion>
    OwnPtr<AffineTransform> m_supplementalTransform;
};

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGStyledTransformableElement_h
