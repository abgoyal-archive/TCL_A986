

#ifndef CanvasGradient_h
#define CanvasGradient_h

#include "Gradient.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class String;

    typedef int ExceptionCode;

    class CanvasGradient : public RefCounted<CanvasGradient> {
    public:
        static PassRefPtr<CanvasGradient> create(const FloatPoint& p0, const FloatPoint& p1)
        {
            return adoptRef(new CanvasGradient(p0, p1));
        }
        static PassRefPtr<CanvasGradient> create(const FloatPoint& p0, float r0, const FloatPoint& p1, float r1)
        {
            return adoptRef(new CanvasGradient(p0, r0, p1, r1));
        }
        
        Gradient* gradient() const { return m_gradient.get(); }

        void addColorStop(float value, const String& color, ExceptionCode&);

        void getColor(float value, float* r, float* g, float* b, float* a) const { m_gradient->getColor(value, r, g, b, a); }

#if ENABLE(DASHBOARD_SUPPORT)
        void setDashboardCompatibilityMode() { m_dashbardCompatibilityMode = true; }
#endif

    private:
        CanvasGradient(const FloatPoint& p0, const FloatPoint& p1);
        CanvasGradient(const FloatPoint& p0, float r0, const FloatPoint& p1, float r1);
        
        RefPtr<Gradient> m_gradient;
        bool m_dashbardCompatibilityMode;
    };

} //namespace

#endif
