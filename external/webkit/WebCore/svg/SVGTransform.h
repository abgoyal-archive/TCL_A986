

#ifndef SVGTransform_h
#define SVGTransform_h

#if ENABLE(SVG)
#include "AffineTransform.h"
#include "FloatPoint.h"
#include "SVGNames.h"

namespace WebCore {
    
    class FloatSize;

    class SVGTransform {
    public:
        enum SVGTransformType {
            SVG_TRANSFORM_UNKNOWN           = 0,
            SVG_TRANSFORM_MATRIX            = 1,
            SVG_TRANSFORM_TRANSLATE         = 2,
            SVG_TRANSFORM_SCALE             = 3,
            SVG_TRANSFORM_ROTATE            = 4,
            SVG_TRANSFORM_SKEWX             = 5,
            SVG_TRANSFORM_SKEWY             = 6
        };
 
        SVGTransform();
        SVGTransform(SVGTransformType);
        explicit SVGTransform(const AffineTransform&);
        virtual ~SVGTransform();
               
        SVGTransformType type() const;

        AffineTransform matrix() const;
    
        float angle() const;
        FloatPoint rotationCenter() const;

        void setMatrix(AffineTransform);

        void setTranslate(float tx, float ty);
        void setScale(float sx, float sy);
        void setRotate(float angle, float cx, float cy);
        void setSkewX(float angle);
        void setSkewY(float angle);
        
        // Internal use only (animation system)
        FloatPoint translate() const;
        FloatSize scale() const;
        
        bool isValid();

    private:
        SVGTransformType m_type;
        float m_angle;
        FloatPoint m_center;
        AffineTransform m_matrix;
    };

    inline bool operator==(const SVGTransform& a, const SVGTransform& b)
    {
        return a.type() == b.type() && a.angle() == b.angle() && a.matrix() == b.matrix();
    }
    
    inline bool operator!=(const SVGTransform& a, const SVGTransform& b)
    {
        return !(a == b);
    }
    
} // namespace WebCore

#endif // ENABLE(SVG)
#endif
