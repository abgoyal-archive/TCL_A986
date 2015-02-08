

#ifndef DataRef_h
#define DataRef_h

#include <wtf/RefPtr.h>

namespace WebCore {

template <typename T> class DataRef {
public:
    const T* get() const { return m_data.get(); }

    const T& operator*() const { return *get(); }
    const T* operator->() const { return get(); }

    T* access()
    {
        if (!m_data->hasOneRef())
            m_data = m_data->copy();
        return m_data.get();
    }

    void init()
    {
        ASSERT(!m_data);
        m_data = T::create();
    }

    bool operator==(const DataRef<T>& o) const
    {
        ASSERT(m_data);
        ASSERT(o.m_data);
        return m_data == o.m_data || *m_data == *o.m_data;
    }
    
    bool operator!=(const DataRef<T>& o) const
    {
        ASSERT(m_data);
        ASSERT(o.m_data);
        return m_data != o.m_data && *m_data != *o.m_data;
    }

private:
    RefPtr<T> m_data;
};

} // namespace WebCore

#endif // DataRef_h
