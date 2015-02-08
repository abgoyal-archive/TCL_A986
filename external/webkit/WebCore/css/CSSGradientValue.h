

#ifndef CSSGradientValue_h
#define CSSGradientValue_h

#include "CSSImageGeneratorValue.h"
#include "CSSPrimitiveValue.h"
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class FloatPoint;
class Gradient;

enum CSSGradientType { CSSLinearGradient, CSSRadialGradient };

struct CSSGradientColorStop {
    CSSGradientColorStop()
        : m_stop(0)
    {
    }
    
    float m_stop;
    RefPtr<CSSPrimitiveValue> m_color;
};

class CSSGradientValue : public CSSImageGeneratorValue {
public:
    static PassRefPtr<CSSGradientValue> create()
    {
        return adoptRef(new CSSGradientValue);
    }

    virtual String cssText() const;

    virtual Image* image(RenderObject*, const IntSize&);

    CSSGradientType type() const { return m_type; }
    void setType(CSSGradientType type) { m_type = type; }
    
    void setFirstX(PassRefPtr<CSSPrimitiveValue> val) { m_firstX = val; }
    void setFirstY(PassRefPtr<CSSPrimitiveValue> val) { m_firstY = val; }
    void setSecondX(PassRefPtr<CSSPrimitiveValue> val) { m_secondX = val; }
    void setSecondY(PassRefPtr<CSSPrimitiveValue> val) { m_secondY = val; }
    
    void setFirstRadius(PassRefPtr<CSSPrimitiveValue> val) { m_firstRadius = val; }
    void setSecondRadius(PassRefPtr<CSSPrimitiveValue> val) { m_secondRadius = val; }

    void addStop(const CSSGradientColorStop& stop) { m_stops.append(stop); }

    void sortStopsIfNeeded();

private:
    CSSGradientValue()
        : m_type(CSSLinearGradient)
        , m_stopsSorted(false)
    {
    }
    
    // Create the gradient for a given size.
    PassRefPtr<Gradient> createGradient(RenderObject*, const IntSize&);
    
    // Resolve points/radii to front end values.
    FloatPoint resolvePoint(CSSPrimitiveValue*, CSSPrimitiveValue*, const IntSize&, float zoomFactor);
    float resolveRadius(CSSPrimitiveValue*, float zoomFactor);
    
    // Type
    CSSGradientType m_type;

    // Points
    RefPtr<CSSPrimitiveValue> m_firstX;
    RefPtr<CSSPrimitiveValue> m_firstY;
    
    RefPtr<CSSPrimitiveValue> m_secondX;
    RefPtr<CSSPrimitiveValue> m_secondY;
    
    // Radii (for radial gradients only)
    RefPtr<CSSPrimitiveValue> m_firstRadius;
    RefPtr<CSSPrimitiveValue> m_secondRadius;
    
    // Stops
    Vector<CSSGradientColorStop> m_stops;
    bool m_stopsSorted;
};

} // namespace WebCore

#endif // CSSGradientValue_h
