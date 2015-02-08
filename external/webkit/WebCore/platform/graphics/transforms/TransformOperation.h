

#ifndef TransformOperation_h
#define TransformOperation_h

#include "TransformationMatrix.h"
#include "IntSize.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

// CSS Transforms (may become part of CSS3)

class TransformOperation : public RefCounted<TransformOperation> {
public:
    enum OperationType {
        SCALE_X, SCALE_Y, SCALE, 
        TRANSLATE_X, TRANSLATE_Y, TRANSLATE, 
        ROTATE,
        ROTATE_Z = ROTATE,
        SKEW_X, SKEW_Y, SKEW, 
        MATRIX,
        SCALE_Z, SCALE_3D,
        TRANSLATE_Z, TRANSLATE_3D,
        ROTATE_X, ROTATE_Y, ROTATE_3D,
        MATRIX_3D,
        PERSPECTIVE,
        IDENTITY, NONE
    };

    virtual ~TransformOperation() { }

    virtual bool operator==(const TransformOperation&) const = 0;
    bool operator!=(const TransformOperation& o) const { return !(*this == o); }

    virtual bool isIdentity() const = 0;

    // Return true if the borderBoxSize was used in the computation, false otherwise.
    virtual bool apply(TransformationMatrix&, const IntSize& borderBoxSize) const = 0;

    virtual PassRefPtr<TransformOperation> blend(const TransformOperation* from, double progress, bool blendToIdentity = false) = 0;

    virtual OperationType getOperationType() const = 0;
    virtual bool isSameType(const TransformOperation&) const { return false; }
    
    bool is3DOperation() const
    {
        OperationType opType = getOperationType();
        return opType == SCALE_Z ||
               opType == SCALE_3D ||
               opType == TRANSLATE_Z ||
               opType == TRANSLATE_3D ||
               opType == ROTATE_X ||
               opType == ROTATE_Y ||
               opType == ROTATE_3D ||
               opType == MATRIX_3D ||
               opType == PERSPECTIVE;
    }
};

} // namespace WebCore

#endif // TransformOperation_h
