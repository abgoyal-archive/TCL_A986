

#ifndef TranslateTransformOperation_h
#define TranslateTransformOperation_h

#include "Length.h"
#include "TransformOperation.h"

namespace WebCore {

class TranslateTransformOperation : public TransformOperation {
public:
    static PassRefPtr<TranslateTransformOperation> create(const Length& tx, const Length& ty, OperationType type)
    {
        return adoptRef(new TranslateTransformOperation(tx, ty, Length(0, Fixed), type));
    }

    static PassRefPtr<TranslateTransformOperation> create(const Length& tx, const Length& ty, const Length& tz, OperationType type)
    {
        return adoptRef(new TranslateTransformOperation(tx, ty, tz, type));
    }

    double x(const IntSize& borderBoxSize) const { return m_x.calcFloatValue(borderBoxSize.width()); }
    double y(const IntSize& borderBoxSize) const { return m_y.calcFloatValue(borderBoxSize.height()); }
    double z(const IntSize&) const { return m_z.calcFloatValue(1); }

private:
    virtual bool isIdentity() const { return m_x.calcFloatValue(1) == 0 && m_y.calcFloatValue(1) == 0 && m_z.calcFloatValue(1) == 0; }

    virtual OperationType getOperationType() const { return m_type; }
    virtual bool isSameType(const TransformOperation& o) const { return o.getOperationType() == m_type; }

    virtual bool operator==(const TransformOperation& o) const
    {
        if (!isSameType(o))
            return false;
        const TranslateTransformOperation* t = static_cast<const TranslateTransformOperation*>(&o);
        return m_x == t->m_x && m_y == t->m_y && m_z == t->m_z;
    }

    virtual bool apply(TransformationMatrix& transform, const IntSize& borderBoxSize) const
    {
        transform.translate3d(x(borderBoxSize), y(borderBoxSize), z(borderBoxSize));
        return m_x.type() == Percent || m_y.type() == Percent;
    }

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation* from, double progress, bool blendToIdentity = false);

    TranslateTransformOperation(const Length& tx, const Length& ty, const Length& tz, OperationType type)
        : m_x(tx)
        , m_y(ty)
        , m_z(tz)
        , m_type(type)
    {
        ASSERT(type == TRANSLATE_X || type == TRANSLATE_Y || type == TRANSLATE_Z || type == TRANSLATE || type == TRANSLATE_3D);
    }

    Length m_x;
    Length m_y;
    Length m_z;
    OperationType m_type;
};

} // namespace WebCore

#endif // TranslateTransformOperation_h
