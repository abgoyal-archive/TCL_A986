

#ifndef MallocZoneSupport_h
#define MallocZoneSupport_h

#include <malloc/malloc.h>

namespace WTF {

class RemoteMemoryReader {
    task_t m_task;
    memory_reader_t* m_reader;

public:
    RemoteMemoryReader(task_t task, memory_reader_t* reader)
        : m_task(task)
        , m_reader(reader)
    { }

    void* operator()(vm_address_t address, size_t size) const
    {
        void* output;
        kern_return_t err = (*m_reader)(m_task, address, size, static_cast<void**>(&output));
        ASSERT(!err);
        if (err)
            output = 0;
        return output;
    }

    template <typename T>
    T* operator()(T* address, size_t size=sizeof(T)) const
    {
        return static_cast<T*>((*this)(reinterpret_cast<vm_address_t>(address), size));
    }
};

} // namespace WTF

#endif // MallocZoneSupport_h
