

#ifndef StructureChain_h
#define StructureChain_h

#include <wtf/OwnArrayPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace JSC {

    class Structure;

    class StructureChain : public RefCounted<StructureChain> {
        friend class JIT;

    public:
        static PassRefPtr<StructureChain> create(Structure* head) { return adoptRef(new StructureChain(head)); }
        RefPtr<Structure>* head() { return m_vector.get(); }

    private:
        StructureChain(Structure* head);

        OwnArrayPtr<RefPtr<Structure> > m_vector;
    };

} // namespace JSC

#endif // StructureChain_h
