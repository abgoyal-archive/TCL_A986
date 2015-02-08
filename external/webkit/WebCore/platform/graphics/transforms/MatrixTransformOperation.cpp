

#include "config.h"
#include "MatrixTransformOperation.h"

#include <algorithm>

using namespace std;

namespace WebCore {

PassRefPtr<TransformOperation> MatrixTransformOperation::blend(const TransformOperation* from, double progress, bool blendToIdentity)
{
    if (from && !from->isSameType(*this))
        return this;

    // convert the TransformOperations into matrices
    IntSize size;
    TransformationMatrix fromT;
    TransformationMatrix toT(m_a, m_b, m_c, m_d, m_e, m_f);
    if (from) {
        const MatrixTransformOperation* m = static_cast<const MatrixTransformOperation*>(from);
        fromT.setMatrix(m->m_a, m->m_b, m->m_c, m->m_d, m->m_e, m->m_f);
    }
    
    if (blendToIdentity)
        std::swap(fromT, toT);

    toT.blend(fromT, progress);
    return MatrixTransformOperation::create(toT.a(), toT.b(), toT.c(), toT.d(), toT.e(), toT.f());
}

} // namespace WebCore
