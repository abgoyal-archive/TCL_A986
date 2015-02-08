

#include "config.h"
#include "Matrix3DTransformOperation.h"

#include <algorithm>

using namespace std;

namespace WebCore {

PassRefPtr<TransformOperation> Matrix3DTransformOperation::blend(const TransformOperation* from, double progress, bool blendToIdentity)
{
    if (from && !from->isSameType(*this))
        return this;

    // Convert the TransformOperations into matrices
    IntSize size;
    TransformationMatrix fromT;
    TransformationMatrix toT;
    if (from)
        from->apply(fromT, size);

    apply(toT, size);

    if (blendToIdentity)
        std::swap(fromT, toT);

    toT.blend(fromT, progress);
    return Matrix3DTransformOperation::create(toT);
}

} // namespace WebCore
