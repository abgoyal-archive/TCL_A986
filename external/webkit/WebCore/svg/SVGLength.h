

#ifndef SVGLength_h
#define SVGLength_h

#if ENABLE(SVG)
#include "PlatformString.h"

namespace WebCore {

    enum SVGLengthType {
        LengthTypeUnknown = 0,
        LengthTypeNumber = 1,
        LengthTypePercentage = 2,
        LengthTypeEMS = 3,
        LengthTypeEXS = 4,
        LengthTypePX = 5,
        LengthTypeCM = 6,
        LengthTypeMM = 7,
        LengthTypeIN = 8,
        LengthTypePT = 9,
        LengthTypePC = 10
    };

    enum SVGLengthMode {
        LengthModeWidth = 0,
        LengthModeHeight,
        LengthModeOther
    };

    class SVGElement;

    class SVGLength {
    public:
        // Forward declare these enums in the w3c naming scheme, for IDL generation
        enum {
            SVG_LENGTHTYPE_UNKNOWN = LengthTypeUnknown,
            SVG_LENGTHTYPE_NUMBER = LengthTypeNumber,
            SVG_LENGTHTYPE_PERCENTAGE = LengthTypePercentage,
            SVG_LENGTHTYPE_EMS = LengthTypeEMS,
            SVG_LENGTHTYPE_EXS = LengthTypeEXS,
            SVG_LENGTHTYPE_PX = LengthTypePX,
            SVG_LENGTHTYPE_CM = LengthTypeCM,
            SVG_LENGTHTYPE_MM = LengthTypeMM,
            SVG_LENGTHTYPE_IN = LengthTypeIN,
            SVG_LENGTHTYPE_PT = LengthTypePT,
            SVG_LENGTHTYPE_PC = LengthTypePC
        };

        SVGLength(SVGLengthMode mode = LengthModeOther, const String& valueAsString = String());

        SVGLengthType unitType() const;

        float value(const SVGElement* context) const;
        void setValue(float);

        float valueInSpecifiedUnits() const;
        void setValueInSpecifiedUnits(float);
        
        float valueAsPercentage() const;

        String valueAsString() const;
        bool setValueAsString(const String&);

        void newValueSpecifiedUnits(unsigned short, float valueInSpecifiedUnits);
        void convertToSpecifiedUnits(unsigned short, const SVGElement* context);

        // Helper functions
        static float PercentageOfViewport(float value, const SVGElement* context, SVGLengthMode);

        inline bool isRelative() const
        {
            SVGLengthType type = unitType();
            return (type == LengthTypePercentage || type == LengthTypeEMS || type == LengthTypeEXS);
        }
 
    private:
        float m_valueInSpecifiedUnits;
        unsigned int m_unit;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGLength_h
