

#include "config.h"
#include "SkewTransformOperation.h"

namespace WebCore {

PassRefPtr<TransformOperation> SkewTransformOperation::blend(const TransformOperation* from, double progress, bool blendToIdentity)
{
    if (from && !from->isSameType(*this))
        return this;
    
    if (blendToIdentity)
        return SkewTransformOperation::create(m_angleX - m_angleX * progress, m_angleY - m_angleY * progress, m_type);
    
    const SkewTransformOperation* fromOp = static_cast<const SkewTransformOperation*>(from);
    double fromAngleX = fromOp ? fromOp->m_angleX : 0;
    double fromAngleY = fromOp ? fromOp->m_angleY : 0;
    return SkewTransformOperation::create(fromAngleX + (m_angleX - fromAngleX) * progress, fromAngleY + (m_angleY - fromAngleY) * progress, m_type);
}

} // namespace WebCore
