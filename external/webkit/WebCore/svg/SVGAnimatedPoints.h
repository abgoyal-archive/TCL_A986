

#ifndef SVGAnimatedPoints_h
#define SVGAnimatedPoints_h

#if ENABLE(SVG)

namespace WebCore {
    class SVGPointList;

    class SVGAnimatedPoints {
    public:
        SVGAnimatedPoints();
        virtual ~SVGAnimatedPoints();

        // 'SVGAnimatedPoints' functions
        virtual SVGPointList* points() const = 0;
        virtual SVGPointList* animatedPoints() const = 0;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
