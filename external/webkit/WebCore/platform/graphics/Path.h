

#ifndef Path_h
#define Path_h

#include <algorithm>
#include <wtf/FastAllocBase.h>

#if PLATFORM(CG)
typedef struct CGPath PlatformPath;
#elif PLATFORM(QT)
#include <qpainterpath.h>
typedef QPainterPath PlatformPath;
#elif PLATFORM(WX) && USE(WXGC)
class wxGraphicsPath;
typedef wxGraphicsPath PlatformPath;
#elif PLATFORM(CAIRO)
namespace WebCore {
    struct CairoPath;
}
typedef WebCore::CairoPath PlatformPath;
#elif PLATFORM(SKIA)
class SkPath;
typedef SkPath PlatformPath;
#elif PLATFORM(HAIKU)
class BRegion;
typedef BRegion PlatformPath;
#elif OS(WINCE)
namespace WebCore {
    class PlatformPath;
}
#else
typedef void PlatformPath;
#endif

#if PLATFORM(QT)
/* QPainterPath is valued based */
typedef PlatformPath PlatformPathPtr;
#else
typedef PlatformPath* PlatformPathPtr;
#endif

namespace WebCore {

    class AffineTransform;
    class FloatPoint;
    class FloatRect;
    class FloatSize;
    class GraphicsContext;
    class String;
    class StrokeStyleApplier;

    enum WindRule {
        RULE_NONZERO = 0,
        RULE_EVENODD = 1
    };

    enum PathElementType {
        PathElementMoveToPoint,
        PathElementAddLineToPoint,
        PathElementAddQuadCurveToPoint,
        PathElementAddCurveToPoint,
        PathElementCloseSubpath
    };

    struct PathElement {
        PathElementType type;
        FloatPoint* points;
    };

    typedef void (*PathApplierFunction)(void* info, const PathElement*);

    class Path : public FastAllocBase {
    public:
        Path();
        ~Path();

        Path(const Path&);
        Path& operator=(const Path&);

        void swap(Path& other) { std::swap(m_path, other.m_path); }

        bool contains(const FloatPoint&, WindRule rule = RULE_NONZERO) const;
        bool strokeContains(StrokeStyleApplier*, const FloatPoint&) const;
        FloatRect boundingRect() const;
        FloatRect strokeBoundingRect(StrokeStyleApplier* = 0);
        
        float length();
        FloatPoint pointAtLength(float length, bool& ok);
        float normalAngleAtLength(float length, bool& ok);

        void clear();
        bool isEmpty() const;
        // Gets the current point of the current path, which is conceptually the final point reached by the path so far.
        // Note the Path can be empty (isEmpty() == true) and still have a current point.
        bool hasCurrentPoint() const;

        void moveTo(const FloatPoint&);
        void addLineTo(const FloatPoint&);
        void addQuadCurveTo(const FloatPoint& controlPoint, const FloatPoint& endPoint);
        void addBezierCurveTo(const FloatPoint& controlPoint1, const FloatPoint& controlPoint2, const FloatPoint& endPoint);
        void addArcTo(const FloatPoint&, const FloatPoint&, float radius);
        void closeSubpath();

        void addArc(const FloatPoint&, float radius, float startAngle, float endAngle, bool anticlockwise);
        void addRect(const FloatRect&);
        void addEllipse(const FloatRect&);

        void translate(const FloatSize&);

        String debugString() const;

        PlatformPathPtr platformPath() const { return m_path; }

        static Path createRoundedRectangle(const FloatRect&, const FloatSize& roundingRadii);
        static Path createRoundedRectangle(const FloatRect&, const FloatSize& topLeftRadius, const FloatSize& topRightRadius, const FloatSize& bottomLeftRadius, const FloatSize& bottomRightRadius);
        static Path createRectangle(const FloatRect&);
        static Path createEllipse(const FloatPoint& center, float rx, float ry);
        static Path createCircle(const FloatPoint& center, float r);
        static Path createLine(const FloatPoint&, const FloatPoint&);

        void apply(void* info, PathApplierFunction) const;
        void transform(const AffineTransform&);

    private:
        PlatformPathPtr m_path;
    };

}

#endif
