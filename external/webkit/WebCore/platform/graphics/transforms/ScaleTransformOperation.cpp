

#include "config.h"
#include "ScaleTransformOperation.h"

namespace WebCore {

PassRefPtr<TransformOperation> ScaleTransformOperation::blend(const TransformOperation* from, double progress, bool blendToIdentity)
{
    if (from && !from->isSameType(*this))
        return this;
    
    if (blendToIdentity)
        return ScaleTransformOperation::create(m_x + (1. - m_x) * progress,
                                               m_y + (1. - m_y) * progress,
                                               m_z + (1. - m_z) * progress, m_type);
    
    const ScaleTransformOperation* fromOp = static_cast<const ScaleTransformOperation*>(from);
    double fromX = fromOp ? fromOp->m_x : 1.;
    double fromY = fromOp ? fromOp->m_y : 1.;
    double fromZ = fromOp ? fromOp->m_z : 1.;
    return ScaleTransformOperation::create(fromX + (m_x - fromX) * progress,
                                           fromY + (m_y - fromY) * progress,
                                           fromZ + (m_z - fromZ) * progress, m_type);
}

} // namespace WebCore
