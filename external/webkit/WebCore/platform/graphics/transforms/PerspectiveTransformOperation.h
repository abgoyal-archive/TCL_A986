

#ifndef PerspectiveTransformOperation_h
#define PerspectiveTransformOperation_h

#include "TransformOperation.h"

namespace WebCore {

class PerspectiveTransformOperation : public TransformOperation {
public:
    static PassRefPtr<PerspectiveTransformOperation> create(double p)
    {
        return adoptRef(new PerspectiveTransformOperation(p));
    }
    
private:
    virtual bool isIdentity() const { return m_p == 0; }
    virtual OperationType getOperationType() const { return PERSPECTIVE; }
    virtual bool isSameType(const TransformOperation& o) const { return o.getOperationType() == PERSPECTIVE; }

    virtual bool operator==(const TransformOperation& o) const
    {
        if (!isSameType(o))
            return false;
        const PerspectiveTransformOperation* p = static_cast<const PerspectiveTransformOperation*>(&o);
        return m_p == p->m_p;
    }

    virtual bool apply(TransformationMatrix& transform, const IntSize&) const
    {
        transform.applyPerspective(m_p);
        return false;
    }

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation* from, double progress, bool blendToIdentity = false);

    PerspectiveTransformOperation(double p)
        : m_p(p)
    {
    }

    double m_p;
};

} // namespace WebCore

#endif // PerspectiveTransformOperation_h
