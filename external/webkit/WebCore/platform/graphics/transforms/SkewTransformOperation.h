

#ifndef SkewTransformOperation_h
#define SkewTransformOperation_h

#include "TransformOperation.h"

namespace WebCore {

class SkewTransformOperation : public TransformOperation {
public:
    static PassRefPtr<SkewTransformOperation> create(double angleX, double angleY, OperationType type)
    {
        return adoptRef(new SkewTransformOperation(angleX, angleY, type));
    }

private:
    virtual bool isIdentity() const { return m_angleX == 0 && m_angleY == 0; }
    virtual OperationType getOperationType() const { return m_type; }
    virtual bool isSameType(const TransformOperation& o) const { return o.getOperationType() == m_type; }

    virtual bool operator==(const TransformOperation& o) const
    {
        if (!isSameType(o))
            return false;
        const SkewTransformOperation* s = static_cast<const SkewTransformOperation*>(&o);
        return m_angleX == s->m_angleX && m_angleY == s->m_angleY;
    }

    virtual bool apply(TransformationMatrix& transform, const IntSize&) const
    {
        transform.skew(m_angleX, m_angleY);
        return false;
    }

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation* from, double progress, bool blendToIdentity = false);
    
    SkewTransformOperation(double angleX, double angleY, OperationType type)
        : m_angleX(angleX)
        , m_angleY(angleY)
        , m_type(type)
    {
    }
    
    double m_angleX;
    double m_angleY;
    OperationType m_type;
};

} // namespace WebCore

#endif // SkewTransformOperation_h
