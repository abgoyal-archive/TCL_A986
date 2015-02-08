

#ifndef TransformState_h
#define TransformState_h

#include "AffineTransform.h"
#include "FloatPoint.h"
#include "FloatQuad.h"
#include "IntSize.h"
#include "TransformationMatrix.h"
#include <wtf/Noncopyable.h>
#include <wtf/PassRefPtr.h>
#include <wtf/OwnPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class TransformState : public Noncopyable {
public:
    enum TransformDirection { ApplyTransformDirection, UnapplyInverseTransformDirection };
    enum TransformAccumulation { FlattenTransform, AccumulateTransform };

    // If quad is non-null, it will be mapped
    TransformState(TransformDirection mappingDirection, const FloatPoint& p, const FloatQuad* quad = 0)
        : m_lastPlanarPoint(p)
        , m_accumulatingTransform(false)
        , m_mapQuad(quad != 0)
        , m_direction(mappingDirection)
    {
        if (quad)
            m_lastPlanarQuad = *quad;
    }
    
    void move(const IntSize& s, TransformAccumulation accumulate = FlattenTransform)
    {
        move(s.width(), s.height(), accumulate);
    }
    
    void move(int x, int y, TransformAccumulation = FlattenTransform);
    void applyTransform(const AffineTransform& transformFromContainer, TransformAccumulation = FlattenTransform);
    void applyTransform(const TransformationMatrix& transformFromContainer, TransformAccumulation = FlattenTransform);
    void flatten();

    // Return the coords of the point or quad in the last flattened layer
    FloatPoint lastPlanarPoint() const { return m_lastPlanarPoint; }
    FloatQuad lastPlanarQuad() const { return m_lastPlanarQuad; }

    // Return the point or quad mapped through the current transform
    FloatPoint mappedPoint() const;
    FloatQuad mappedQuad() const;

private:
    void flattenWithTransform(const TransformationMatrix&);
    
    FloatPoint m_lastPlanarPoint;
    FloatQuad m_lastPlanarQuad;

    // We only allocate the transform if we need to
    OwnPtr<TransformationMatrix> m_accumulatedTransform;
    bool m_accumulatingTransform;
    bool m_mapQuad;
    TransformDirection m_direction;
};

class HitTestingTransformState : public RefCounted<HitTestingTransformState> {
public:
    static PassRefPtr<HitTestingTransformState> create(const FloatPoint& p, const FloatQuad& quad)
    {
        return adoptRef(new HitTestingTransformState(p, quad));
    }

    static PassRefPtr<HitTestingTransformState> create(const HitTestingTransformState& other)
    {
        return adoptRef(new HitTestingTransformState(other));
    }

    enum TransformAccumulation { FlattenTransform, AccumulateTransform };
    void translate(int x, int y, TransformAccumulation);
    void applyTransform(const TransformationMatrix& transformFromContainer, TransformAccumulation);

    FloatPoint mappedPoint() const;
    FloatQuad mappedQuad() const;
    void flatten();

    FloatPoint m_lastPlanarPoint;
    FloatQuad m_lastPlanarQuad;
    TransformationMatrix m_accumulatedTransform;
    bool m_accumulatingTransform;

private:
    HitTestingTransformState(const FloatPoint& p, const FloatQuad& quad)
        : m_lastPlanarPoint(p)
        , m_lastPlanarQuad(quad)
        , m_accumulatingTransform(false)
    {
    }
    
    HitTestingTransformState(const HitTestingTransformState& other)
        : RefCounted<HitTestingTransformState>()
        , m_lastPlanarPoint(other.m_lastPlanarPoint)
        , m_lastPlanarQuad(other.m_lastPlanarQuad)
        , m_accumulatedTransform(other.m_accumulatedTransform)
        , m_accumulatingTransform(other.m_accumulatingTransform)
    {
    }
    
    void flattenWithTransform(const TransformationMatrix&);
};

} // namespace WebCore

#endif // TransformState_h
