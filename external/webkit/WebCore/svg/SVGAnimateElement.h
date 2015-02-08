

#ifndef SVGAnimateElement_h
#define SVGAnimateElement_h

#if ENABLE(SVG) && ENABLE(SVG_ANIMATION)

#include "Color.h"
#include "SVGAnimationElement.h"

namespace WebCore {
    class SVGPathSegList;

    class SVGAnimateElement : public SVGAnimationElement {
    public:
        SVGAnimateElement(const QualifiedName&, Document*);
        virtual ~SVGAnimateElement();
    
    protected:
        virtual void resetToBaseValue(const String&);
        virtual bool calculateFromAndToValues(const String& fromString, const String& toString);
        virtual bool calculateFromAndByValues(const String& fromString, const String& byString);
        virtual void calculateAnimatedValue(float percentage, unsigned repeat, SVGSMILElement* resultElement);
        virtual void applyResultsToTarget();
        virtual float calculateDistance(const String& fromString, const String& toString);

    private:
        enum PropertyType { NumberProperty, ColorProperty, StringProperty, PathProperty };
        PropertyType determinePropertyType(const String& attribute) const;
        PropertyType m_propertyType;
        
        double m_fromNumber;
        double m_toNumber;
        double m_animatedNumber;
        String m_numberUnit;
        Color m_fromColor;
        Color m_toColor;
        Color m_animatedColor;
        String m_fromString;
        String m_toString;
        String m_animatedString;
        RefPtr<SVGPathSegList> m_fromPath;
        RefPtr<SVGPathSegList> m_toPath;
        RefPtr<SVGPathSegList> m_animatedPath;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGAnimateElement_h

// vim:ts=4:noet
