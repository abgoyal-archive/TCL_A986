

#ifndef SVGAngle_h
#define SVGAngle_h

#if ENABLE(SVG)
#include "PlatformString.h"
#include "SVGNames.h"

namespace WebCore {

    class SVGAngle {
    public:
        SVGAngle();
        virtual ~SVGAngle();

        enum SVGAngleType {
            SVG_ANGLETYPE_UNKNOWN           = 0,
            SVG_ANGLETYPE_UNSPECIFIED       = 1,
            SVG_ANGLETYPE_DEG               = 2,
            SVG_ANGLETYPE_RAD               = 3,
            SVG_ANGLETYPE_GRAD              = 4
        };

        SVGAngleType unitType() const;

        void setValue(float);
        float value() const; 

        void setValueInSpecifiedUnits(float valueInSpecifiedUnits);
        float valueInSpecifiedUnits() const;

        void setValueAsString(const String&);
        String valueAsString() const;

        void newValueSpecifiedUnits(unsigned short unitType, float valueInSpecifiedUnits);
        void convertToSpecifiedUnits(unsigned short unitType);

    private:
        SVGAngleType m_unitType;
        float m_value;
        float m_valueInSpecifiedUnits;
        mutable String m_valueAsString;

        void calculate();
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGAngle_h
