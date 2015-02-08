

#ifndef ByteArray_h
#define ByteArray_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WTF {
    class ByteArray : public RefCountedBase {
    public:
        unsigned length() const { return m_size; }

        void set(unsigned index, double value)
        {
            if (index >= m_size)
                return;
            if (!(value > 0)) // Clamp NaN to 0
                value = 0;
            else if (value > 255)
                value = 255;
            m_data[index] = static_cast<unsigned char>(value + 0.5);
        }

        void set(unsigned index, unsigned char value)
        {
            if (index >= m_size)
                return;
            m_data[index] = value;
        }

        bool get(unsigned index, unsigned char& result) const
        {
            if (index >= m_size)
                return false;
            result = m_data[index];
            return true;
        }

        unsigned char get(unsigned index) const
        {
            ASSERT(index < m_size);
            return m_data[index];
        }

        unsigned char* data() { return m_data; }

        void deref()
        {
            if (derefBase()) {
                // We allocated with new unsigned char[] in create(),
                // and then used placement new to construct the object.
                this->~ByteArray();
                delete[] reinterpret_cast<unsigned char*>(this);
            }
        }

        static PassRefPtr<ByteArray> create(size_t size);

    private:
        ByteArray(size_t size)
            : m_size(size)
        {
        }
        size_t m_size;
        unsigned char m_data[sizeof(size_t)];
    };
}

#endif
