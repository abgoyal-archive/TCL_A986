

#ifndef RotateTransformOperation_h
#define RotateTransformOperation_h

#include "TransformOperation.h"

namespace WebCore {

class RotateTransformOperation : public TransformOperation {
public:
    static PassRefPtr<RotateTransformOperation> create(double angle, OperationType type)
    {
        return adoptRef(new RotateTransformOperation(0, 0, 1, angle, type));
    }

    static PassRefPtr<RotateTransformOperation> create(double x, double y, double z, double angle, OperationType type)
    {
        return adoptRef(new RotateTransformOperation(x, y, z, angle, type));
    }

    double angle() const { return m_angle; }

private:
    virtual bool isIdentity() const { return m_angle == 0; }

    virtual OperationType getOperationType() const { return m_type; }
    virtual bool isSameType(const TransformOperation& o) const { return o.getOperationType() == m_type; }

    virtual bool operator==(const TransformOperation& o) const
    {
        if (!isSameType(o))
            return false;
        const RotateTransformOperation* r = static_cast<const RotateTransformOperation*>(&o);
        return m_x == r->m_x && m_y == r->m_y && m_z == r->m_z && m_angle == r->m_angle;
    }

    virtual bool apply(TransformationMatrix& transform, const IntSize& /*borderBoxSize*/) const
    {
        transform.rotate3d(m_x, m_y, m_z, m_angle);
        return false;
    }

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation* from, double progress, bool blendToIdentity = false);

    RotateTransformOperation(double x, double y, double z, double angle, OperationType type)
        : m_x(x)
        , m_y(y)
        , m_z(z)
        , m_angle(angle)
        , m_type(type)
    {
        ASSERT(type == ROTATE_X || type == ROTATE_Y || type == ROTATE_Z || type == ROTATE_3D);
    }

    double m_x;
    double m_y;
    double m_z;
    double m_angle;
    OperationType m_type;
};

} // namespace WebCore

#endif // RotateTransformOperation_h
