

#ifndef ScaleTransformOperation_h
#define ScaleTransformOperation_h

#include "TransformOperation.h"

namespace WebCore {

class ScaleTransformOperation : public TransformOperation {
public:
    static PassRefPtr<ScaleTransformOperation> create(double sx, double sy, OperationType type)
    {
        return adoptRef(new ScaleTransformOperation(sx, sy, 1, type));
    }

    static PassRefPtr<ScaleTransformOperation> create(double sx, double sy, double sz, OperationType type)
    {
        return adoptRef(new ScaleTransformOperation(sx, sy, sz, type));
    }

    double x() const { return m_x; }
    double y() const { return m_y; }
    double z() const { return m_z; }

private:
    virtual bool isIdentity() const { return m_x == 1 &&  m_y == 1 &&  m_z == 1; }

    virtual OperationType getOperationType() const { return m_type; }
    virtual bool isSameType(const TransformOperation& o) const { return o.getOperationType() == m_type; }

    virtual bool operator==(const TransformOperation& o) const
    {
        if (!isSameType(o))
            return false;
        const ScaleTransformOperation* s = static_cast<const ScaleTransformOperation*>(&o);
        return m_x == s->m_x && m_y == s->m_y && m_z == s->m_z;
    }

    virtual bool apply(TransformationMatrix& transform, const IntSize&) const
    {
        transform.scale3d(m_x, m_y, m_z);
        return false;
    }

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation* from, double progress, bool blendToIdentity = false);

    ScaleTransformOperation(double sx, double sy, double sz, OperationType type)
        : m_x(sx)
        , m_y(sy)
        , m_z(sz)
        , m_type(type)
    {
        ASSERT(type == SCALE_X || type == SCALE_Y || type == SCALE_Z || type == SCALE || type == SCALE_3D);
    }
        
    double m_x;
    double m_y;
    double m_z;
    OperationType m_type;
};

} // namespace WebCore

#endif // ScaleTransformOperation_h
