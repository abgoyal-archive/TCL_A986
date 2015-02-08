

#ifndef IdentityTransformOperation_h
#define IdentityTransformOperation_h

#include "TransformOperation.h"

namespace WebCore {

class IdentityTransformOperation : public TransformOperation {
public:
    static PassRefPtr<IdentityTransformOperation> create()
    {
        return adoptRef(new IdentityTransformOperation());
    }
        
private:
    virtual bool isIdentity() const { return true; }
    virtual OperationType getOperationType() const { return IDENTITY; }
    virtual bool isSameType(const TransformOperation& o) const { return o.getOperationType() == IDENTITY; }

    virtual bool operator==(const TransformOperation& o) const
    {
        return isSameType(o);
    }

    virtual bool apply(TransformationMatrix&, const IntSize&) const
    {
        return false;
    }

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation*, double, bool = false)
    {
        return this;
    }

    IdentityTransformOperation()
    {
    }

};

} // namespace WebCore

#endif // IdentityTransformOperation_h
