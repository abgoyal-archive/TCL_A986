

#ifndef SVGPathElement_h
#define SVGPathElement_h

#if ENABLE(SVG)
#include "SVGAnimatedPathData.h"
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"

namespace WebCore {

    class SVGPathSeg;
    class SVGPathSegArcAbs;
    class SVGPathSegArcRel;
    class SVGPathSegClosePath;
    class SVGPathSegLinetoAbs;
    class SVGPathSegLinetoRel;
    class SVGPathSegMovetoAbs;
    class SVGPathSegMovetoRel;
    class SVGPathSegCurvetoCubicAbs;
    class SVGPathSegCurvetoCubicRel;
    class SVGPathSegLinetoVerticalAbs;
    class SVGPathSegLinetoVerticalRel;
    class SVGPathSegLinetoHorizontalAbs;
    class SVGPathSegLinetoHorizontalRel;
    class SVGPathSegCurvetoQuadraticAbs;
    class SVGPathSegCurvetoQuadraticRel;
    class SVGPathSegCurvetoCubicSmoothAbs;
    class SVGPathSegCurvetoCubicSmoothRel;
    class SVGPathSegCurvetoQuadraticSmoothAbs;
    class SVGPathSegCurvetoQuadraticSmoothRel;
    class SVGPathElement : public SVGStyledTransformableElement,
                           public SVGTests,
                           public SVGLangSpace,
                           public SVGExternalResourcesRequired,
                           public SVGAnimatedPathData {
    public:
        SVGPathElement(const QualifiedName&, Document*);
        virtual ~SVGPathElement();
        
        virtual bool isValid() const { return SVGTests::isValid(); }
        float getTotalLength();
        FloatPoint getPointAtLength(float distance);
        unsigned long getPathSegAtLength(float distance, ExceptionCode&);

        static PassRefPtr<SVGPathSegClosePath> createSVGPathSegClosePath();
        static PassRefPtr<SVGPathSegMovetoAbs> createSVGPathSegMovetoAbs(float x, float y);
        static PassRefPtr<SVGPathSegMovetoRel> createSVGPathSegMovetoRel(float x, float y);
        static PassRefPtr<SVGPathSegLinetoAbs> createSVGPathSegLinetoAbs(float x, float y);
        static PassRefPtr<SVGPathSegLinetoRel> createSVGPathSegLinetoRel(float x, float y);
        static PassRefPtr<SVGPathSegCurvetoCubicAbs> createSVGPathSegCurvetoCubicAbs(float x, float y, float x1, float y1, float x2, float y2);
        static PassRefPtr<SVGPathSegCurvetoCubicRel> createSVGPathSegCurvetoCubicRel(float x, float y, float x1, float y1, float x2, float y2);
        static PassRefPtr<SVGPathSegCurvetoQuadraticAbs> createSVGPathSegCurvetoQuadraticAbs(float x, float y, float x1, float y1);
        static PassRefPtr<SVGPathSegCurvetoQuadraticRel> createSVGPathSegCurvetoQuadraticRel(float x, float y, float x1, float y1);
        static PassRefPtr<SVGPathSegArcAbs> createSVGPathSegArcAbs(float x, float y, float r1, float r2, float angle, bool largeArcFlag, bool sweepFlag);
        static PassRefPtr<SVGPathSegArcRel> createSVGPathSegArcRel(float x, float y, float r1, float r2, float angle, bool largeArcFlag, bool sweepFlag);
        static PassRefPtr<SVGPathSegLinetoHorizontalAbs> createSVGPathSegLinetoHorizontalAbs(float x);
        static PassRefPtr<SVGPathSegLinetoHorizontalRel> createSVGPathSegLinetoHorizontalRel(float x);
        static PassRefPtr<SVGPathSegLinetoVerticalAbs> createSVGPathSegLinetoVerticalAbs(float y);
        static PassRefPtr<SVGPathSegLinetoVerticalRel> createSVGPathSegLinetoVerticalRel(float y);
        static PassRefPtr<SVGPathSegCurvetoCubicSmoothAbs> createSVGPathSegCurvetoCubicSmoothAbs(float x, float y, float x2, float y2);
        static PassRefPtr<SVGPathSegCurvetoCubicSmoothRel> createSVGPathSegCurvetoCubicSmoothRel(float x, float y, float x2, float y2);
        static PassRefPtr<SVGPathSegCurvetoQuadraticSmoothAbs> createSVGPathSegCurvetoQuadraticSmoothAbs(float x, float y);
        static PassRefPtr<SVGPathSegCurvetoQuadraticSmoothRel> createSVGPathSegCurvetoQuadraticSmoothRel(float x, float y);

        // Derived from: 'SVGAnimatedPathData'
        virtual SVGPathSegList* pathSegList() const;
        virtual SVGPathSegList* normalizedPathSegList() const;
        virtual SVGPathSegList* animatedPathSegList() const;
        virtual SVGPathSegList* animatedNormalizedPathSegList() const;

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual Path toPathData() const;

        virtual bool supportsMarkers() const { return true; }

    private:
        mutable RefPtr<SVGPathSegList> m_pathSegList;

        DECLARE_ANIMATED_PROPERTY(SVGPathElement, SVGNames::pathLengthAttr, float, PathLength, pathLength)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGPathElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
