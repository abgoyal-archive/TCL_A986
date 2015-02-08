

#ifndef Matrix3DTransformOperation_h
#define Matrix3DTransformOperation_h

#include "TransformOperation.h"

namespace WebCore {

class Matrix3DTransformOperation : public TransformOperation {
public:
    static PassRefPtr<Matrix3DTransformOperation> create(const TransformationMatrix& matrix)
    {
        return adoptRef(new Matrix3DTransformOperation(matrix));
    }

private:    
    virtual bool isIdentity() const { return m_matrix.isIdentity(); }

    virtual OperationType getOperationType() const { return MATRIX_3D; }
    virtual bool isSameType(const TransformOperation& o) const { return o.getOperationType() == MATRIX_3D; }

    virtual bool operator==(const TransformOperation& o) const
    {
        if (!isSameType(o))
            return false;
        const Matrix3DTransformOperation* m = static_cast<const Matrix3DTransformOperation*>(&o);
        return m_matrix == m->m_matrix;
    }

    virtual bool apply(TransformationMatrix& transform, const IntSize&) const
    {
        transform.multLeft(TransformationMatrix(m_matrix));
        return false;
    }

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation* from, double progress, bool blendToIdentity = false);
    
    Matrix3DTransformOperation(const TransformationMatrix& mat)
    {
        m_matrix = mat;
    }

    TransformationMatrix m_matrix;
};

} // namespace WebCore

#endif // Matrix3DTransformOperation_h
