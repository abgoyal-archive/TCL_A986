

#include "config.h"
#include "TransformOperations.h"

#include "IdentityTransformOperation.h"

namespace WebCore {

TransformOperations::TransformOperations(bool makeIdentity)
{
    if (makeIdentity)
        m_operations.append(IdentityTransformOperation::create());
}

bool TransformOperations::operator==(const TransformOperations& o) const
{
    if (m_operations.size() != o.m_operations.size())
        return false;
        
    unsigned s = m_operations.size();
    for (unsigned i = 0; i < s; i++) {
        if (*m_operations[i] != *o.m_operations[i])
            return false;
    }
    
    return true;
}

} // namespace WebCore
