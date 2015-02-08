

#ifndef SVGAnimatedPathData_h
#define SVGAnimatedPathData_h

#if ENABLE(SVG)

namespace WebCore {
    class SVGPathSegList;

    class SVGAnimatedPathData {
    public:
        SVGAnimatedPathData();
        virtual ~SVGAnimatedPathData();

        // 'SVGAnimatedPathData' functions
        virtual SVGPathSegList* pathSegList() const = 0;
        virtual SVGPathSegList* normalizedPathSegList() const = 0;
        virtual SVGPathSegList* animatedPathSegList() const = 0;
        virtual SVGPathSegList* animatedNormalizedPathSegList() const = 0;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
