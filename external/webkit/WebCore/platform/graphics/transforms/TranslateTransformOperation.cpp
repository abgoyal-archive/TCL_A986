

#include "config.h"
#include "TranslateTransformOperation.h"

namespace WebCore {

PassRefPtr<TransformOperation> TranslateTransformOperation::blend(const TransformOperation* from, double progress, bool blendToIdentity)
{
    if (from && !from->isSameType(*this))
        return this;
    
    if (blendToIdentity)
        return TranslateTransformOperation::create(Length(m_x.type()).blend(m_x, progress), 
                                                   Length(m_y.type()).blend(m_y, progress), 
                                                   Length(m_z.type()).blend(m_z, progress), m_type);

    const TranslateTransformOperation* fromOp = static_cast<const TranslateTransformOperation*>(from);
    Length fromX = fromOp ? fromOp->m_x : Length(m_x.type());
    Length fromY = fromOp ? fromOp->m_y : Length(m_y.type());
    Length fromZ = fromOp ? fromOp->m_z : Length(m_z.type());
    return TranslateTransformOperation::create(m_x.blend(fromX, progress), m_y.blend(fromY, progress), m_z.blend(fromZ, progress), m_type);
}

} // namespace WebCore
