

#ifndef MatrixTransformOperation_h
#define MatrixTransformOperation_h

#include "TransformOperation.h"
#include "TransformationMatrix.h"

namespace WebCore {

class MatrixTransformOperation : public TransformOperation {
public:
    static PassRefPtr<MatrixTransformOperation> create(double a, double b, double c, double d, double e, double f)
    {
        return adoptRef(new MatrixTransformOperation(a, b, c, d, e, f));
    }

    static PassRefPtr<MatrixTransformOperation> create(const TransformationMatrix& t)
    {
        return adoptRef(new MatrixTransformOperation(t));
    }

private:
    virtual bool isIdentity() const { return m_a == 1 && m_b == 0 && m_c == 0 && m_d == 1 && m_e == 0 && m_f == 0; }

    virtual OperationType getOperationType() const { return MATRIX; }
    virtual bool isSameType(const TransformOperation& o) const { return o.getOperationType() == MATRIX; }

    virtual bool operator==(const TransformOperation& o) const
    {
        if (!isSameType(o))
            return false;

        const MatrixTransformOperation* m = static_cast<const MatrixTransformOperation*>(&o);
        return m_a == m->m_a && m_b == m->m_b && m_c == m->m_c && m_d == m->m_d && m_e == m->m_e && m_f == m->m_f;
    }

    virtual bool apply(TransformationMatrix& transform, const IntSize&) const
    {
        TransformationMatrix matrix(m_a, m_b, m_c, m_d, m_e, m_f);
        transform.multLeft(TransformationMatrix(matrix));
        return false;
    }

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation* from, double progress, bool blendToIdentity = false);
    
    MatrixTransformOperation(double a, double b, double c, double d, double e, double f)
        : m_a(a)
        , m_b(b)
        , m_c(c)
        , m_d(d)
        , m_e(e)
        , m_f(f)
    {
    }

    MatrixTransformOperation(const TransformationMatrix& t)
        : m_a(t.a())
        , m_b(t.b())
        , m_c(t.c())
        , m_d(t.d())
        , m_e(t.e())
        , m_f(t.f())
    {
    }
    
    double m_a;
    double m_b;
    double m_c;
    double m_d;
    double m_e;
    double m_f;
};

} // namespace WebCore

#endif // MatrixTransformOperation_h
