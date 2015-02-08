

#ifndef SVGAnimateTransformElement_h
#define SVGAnimateTransformElement_h
#if ENABLE(SVG) && ENABLE(SVG_ANIMATION)

#include "SVGAnimationElement.h"
#include "SVGTransform.h"
#include "SVGTransformDistance.h"

namespace WebCore {

class AffineTransform;

class SVGAnimateTransformElement : public SVGAnimationElement {
public:
    SVGAnimateTransformElement(const QualifiedName&, Document*);
    virtual ~SVGAnimateTransformElement();
    
    virtual bool hasValidTarget() const;

    virtual void parseMappedAttribute(MappedAttribute*);

private:
    virtual void resetToBaseValue(const String&);
    virtual bool calculateFromAndToValues(const String& fromString, const String& toString);
    virtual bool calculateFromAndByValues(const String& fromString, const String& byString);
    virtual void calculateAnimatedValue(float percentage, unsigned repeat, SVGSMILElement* resultElement);
    virtual void applyResultsToTarget();
    virtual float calculateDistance(const String& fromString, const String& toString);

    SVGTransform parseTransformValue(const String&) const;
    
    SVGTransform::SVGTransformType m_type;
    
    unsigned m_baseIndexInTransformList;

    SVGTransform m_toTransform;
    SVGTransform m_fromTransform;
};

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGAnimateTransformElement_h

// vim:ts=4:noet
