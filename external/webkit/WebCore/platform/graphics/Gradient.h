

#ifndef Gradient_h
#define Gradient_h

#include "AffineTransform.h"
#include "FloatPoint.h"
#include "Generator.h"
#include "GraphicsTypes.h"
#include <wtf/PassRefPtr.h>
#include <wtf/Vector.h>

#if PLATFORM(CG)

#ifdef BUILDING_ON_TIGER
typedef struct CGShading* CGShadingRef;
typedef CGShadingRef PlatformGradient;
#else
typedef struct CGGradient* CGGradientRef;
typedef CGGradientRef PlatformGradient;
#endif

#elif PLATFORM(QT)
QT_BEGIN_NAMESPACE
class QGradient;
QT_END_NAMESPACE
typedef QGradient* PlatformGradient;
#elif PLATFORM(CAIRO)
typedef struct _cairo_pattern cairo_pattern_t;
typedef cairo_pattern_t* PlatformGradient;
#elif PLATFORM(SKIA)
#if PLATFORM(ANDROID)
#include "SkShader.h"
typedef class PlatformGradientRec* PlatformGradient;
#else
class SkShader;
typedef class SkShader* PlatformGradient;
typedef class SkShader* PlatformPattern;
#endif
#else
typedef void* PlatformGradient;
#endif

namespace WebCore {

    class Color;

    class Gradient : public Generator {
    public:
        static PassRefPtr<Gradient> create(const FloatPoint& p0, const FloatPoint& p1)
        {
            return adoptRef(new Gradient(p0, p1));
        }
        static PassRefPtr<Gradient> create(const FloatPoint& p0, float r0, const FloatPoint& p1, float r1)
        {
            return adoptRef(new Gradient(p0, r0, p1, r1));
        }
        virtual ~Gradient();

        void addColorStop(float, const Color&);

        void getColor(float value, float* r, float* g, float* b, float* a) const;

#if OS(WINCE) && !PLATFORM(QT)
        const FloatPoint& p0() const { return m_p0; }
        const FloatPoint& p1() const { return m_p1; }
        float r0() const { return m_r0; }
        float r1() const { return m_r1; }
        bool isRadial() const { return m_radial; }
        struct ColorStop;
        const Vector<ColorStop>& getStops() const;
#else
#if PLATFORM(ANDROID)
        SkShader* getShader(SkShader::TileMode);
#endif
        PlatformGradient platformGradient();
#endif
        struct ColorStop {
            float stop;
            float red;
            float green;
            float blue;
            float alpha;

            ColorStop() : stop(0), red(0), green(0), blue(0), alpha(0) { }
            ColorStop(float s, float r, float g, float b, float a) : stop(s), red(r), green(g), blue(b), alpha(a) { }
        };

        void setStopsSorted(bool s) { m_stopsSorted = s; }
        
        void setSpreadMethod(GradientSpreadMethod);
        GradientSpreadMethod spreadMethod() { return m_spreadMethod; }
        void setGradientSpaceTransform(const AffineTransform& gradientSpaceTransformation);
        // Qt and CG transform the gradient at draw time
        AffineTransform gradientSpaceTransform() { return m_gradientSpaceTransformation; }

        virtual void fill(GraphicsContext*, const FloatRect&);
        virtual void adjustParametersForTiledDrawing(IntSize& size, FloatRect& srcRect);

        void setPlatformGradientSpaceTransform(const AffineTransform& gradientSpaceTransformation);

#if PLATFORM(CG)
        void paint(GraphicsContext*);
#endif
    private:
        Gradient(const FloatPoint& p0, const FloatPoint& p1);
        Gradient(const FloatPoint& p0, float r0, const FloatPoint& p1, float r1);

        void platformInit() { m_gradient = 0; }
        void platformDestroy();

        int findStop(float value) const;
        void sortStopsIfNecessary();

        bool m_radial;
        FloatPoint m_p0;
        FloatPoint m_p1;
        float m_r0;
        float m_r1;
        mutable Vector<ColorStop> m_stops;
        mutable bool m_stopsSorted;
        mutable int m_lastStop;
        GradientSpreadMethod m_spreadMethod;
        AffineTransform m_gradientSpaceTransformation;

        PlatformGradient m_gradient;
    };

} //namespace

#endif
