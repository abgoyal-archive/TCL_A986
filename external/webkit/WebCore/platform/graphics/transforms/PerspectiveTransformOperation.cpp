

#include "config.h"
#include "PerspectiveTransformOperation.h"

#include <algorithm>

using namespace std;

namespace WebCore {

PassRefPtr<TransformOperation> PerspectiveTransformOperation::blend(const TransformOperation* from, double progress, bool blendToIdentity)
{
    if (from && !from->isSameType(*this))
        return this;
    
    if (blendToIdentity)
        return PerspectiveTransformOperation::create(m_p + (1. - m_p) * progress);
    
    const PerspectiveTransformOperation* fromOp = static_cast<const PerspectiveTransformOperation*>(from);
    double fromP = fromOp ? fromOp->m_p : 0;
    double toP = m_p;

    TransformationMatrix fromT;
    TransformationMatrix toT;
    fromT.applyPerspective(fromP);
    toT.applyPerspective(toP);
    toT.blend(fromT, progress);
    TransformationMatrix::DecomposedType decomp;
    toT.decompose(decomp);
    
    return PerspectiveTransformOperation::create(decomp.perspectiveZ ? -1.0 / decomp.perspectiveZ : 0.0);
}

} // namespace WebCore
