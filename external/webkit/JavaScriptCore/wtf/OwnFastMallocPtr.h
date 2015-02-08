

#ifndef OwnFastMallocPtr_h
#define OwnFastMallocPtr_h

#include "FastMalloc.h"
#include "Noncopyable.h"

namespace WTF {

    template<class T> class OwnFastMallocPtr : public Noncopyable {
    public:
        explicit OwnFastMallocPtr(T* ptr) : m_ptr(ptr)
        {
        }

        ~OwnFastMallocPtr()
        {
            fastFree(m_ptr);
        }

        T* get() const { return m_ptr; }
        T* release() { T* ptr = m_ptr; m_ptr = 0; return ptr; }

    private:
        T* m_ptr;
    };

} // namespace WTF

using WTF::OwnFastMallocPtr;

#endif // OwnFastMallocPtr_h
