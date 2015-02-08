

#ifndef SVGAnimateMotionElement_h
#define SVGAnimateMotionElement_h
#if ENABLE(SVG_ANIMATION)
#include "AffineTransform.h"
#include "Path.h"
#include "SVGAnimationElement.h"

namespace WebCore {
            
    class SVGAnimateMotionElement : public SVGAnimationElement {
    public:
        SVGAnimateMotionElement(const QualifiedName&, Document*);
        virtual ~SVGAnimateMotionElement();

        virtual bool hasValidTarget() const;

        virtual void parseMappedAttribute(MappedAttribute*);
        
    private:
        virtual void resetToBaseValue(const String&);
        virtual bool calculateFromAndToValues(const String& fromString, const String& toString);
        virtual bool calculateFromAndByValues(const String& fromString, const String& byString);
        virtual void calculateAnimatedValue(float percentage, unsigned repeat, SVGSMILElement* resultElement);
        virtual void applyResultsToTarget();
        virtual float calculateDistance(const String& fromString, const String& toString);
        virtual Path animationPath() const;
        
        enum RotateMode {
            RotateAngle,
            RotateAuto,
            RotateAutoReverse
        };
        RotateMode rotateMode() const;

        FloatSize m_animatedTranslation;
        float m_animatedAngle;

        // Note: we do not support percentage values for to/from coords as the spec implies we should (opera doesn't either)
        FloatPoint m_fromPoint;
        float m_fromAngle;
        FloatPoint m_toPoint;
        float m_toAngle;

        unsigned m_baseIndexInTransformList;

        Path m_path;
        Vector<float> m_keyPoints;
        float m_angle;
    };
    
} // namespace WebCore

#endif // ENABLE(SVG_ANIMATION)
#endif // SVGAnimateMotionElement_h

// vim:ts=4:noet
