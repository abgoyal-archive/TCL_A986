

#ifndef TransformOperations_h
#define TransformOperations_h

#include "TransformOperation.h"
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class TransformOperations : public FastAllocBase {
public:
    TransformOperations(bool makeIdentity = false);
    
    bool operator==(const TransformOperations& o) const;
    bool operator!=(const TransformOperations& o) const
    {
        return !(*this == o);
    }
    
    void apply(const IntSize& sz, TransformationMatrix& t) const
    {
        for (unsigned i = 0; i < m_operations.size(); ++i)
            m_operations[i]->apply(t, sz);
    }
    
    // Return true if any of the operation types are 3D operation types (even if the
    // values describe affine transforms)
    bool has3DOperation() const
    {
        for (unsigned i = 0; i < m_operations.size(); ++i)
            if (m_operations[i]->is3DOperation())
                return true;
        return false;
    }
    
    Vector<RefPtr<TransformOperation> >& operations() { return m_operations; }
    const Vector<RefPtr<TransformOperation> >& operations() const { return m_operations; }

    size_t size() const { return m_operations.size(); }
    const TransformOperation* at(size_t index) const { return index < m_operations.size() ? m_operations.at(index).get() : 0; }

private:
    Vector<RefPtr<TransformOperation> > m_operations;
};

} // namespace WebCore

#endif // TransformOperations_h
