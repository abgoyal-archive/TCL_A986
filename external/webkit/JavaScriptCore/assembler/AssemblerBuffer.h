

#ifndef AssemblerBuffer_h
#define AssemblerBuffer_h

#include <wtf/Platform.h>

#if ENABLE(ASSEMBLER)

#include "stdint.h"
#include <string.h>
#include <jit/ExecutableAllocator.h>
#include <wtf/Assertions.h>
#include <wtf/FastMalloc.h>

namespace JSC {

    class AssemblerBuffer {
        static const int inlineCapacity = 256;
    public:
        AssemblerBuffer()
            : m_buffer(m_inlineBuffer)
            , m_capacity(inlineCapacity)
            , m_size(0)
        {
        }

        ~AssemblerBuffer()
        {
            if (m_buffer != m_inlineBuffer)
                fastFree(m_buffer);
        }

        void ensureSpace(int space)
        {
            if (m_size > m_capacity - space)
                grow();
        }

        bool isAligned(int alignment) const
        {
            return !(m_size & (alignment - 1));
        }

        void putByteUnchecked(int value)
        {
            ASSERT(!(m_size > m_capacity - 4));
            m_buffer[m_size] = value;
            m_size++;
        }

        void putByte(int value)
        {
            if (m_size > m_capacity - 4)
                grow();
            putByteUnchecked(value);
        }

        void putShortUnchecked(int value)
        {
            ASSERT(!(m_size > m_capacity - 4));
            *reinterpret_cast<short*>(&m_buffer[m_size]) = value;
            m_size += 2;
        }

        void putShort(int value)
        {
            if (m_size > m_capacity - 4)
                grow();
            putShortUnchecked(value);
        }

        void putIntUnchecked(int value)
        {
            ASSERT(!(m_size > m_capacity - 4));
            *reinterpret_cast<int*>(&m_buffer[m_size]) = value;
            m_size += 4;
        }

        void putInt64Unchecked(int64_t value)
        {
            ASSERT(!(m_size > m_capacity - 8));
            *reinterpret_cast<int64_t*>(&m_buffer[m_size]) = value;
            m_size += 8;
        }

        void putInt(int value)
        {
            if (m_size > m_capacity - 4)
                grow();
            putIntUnchecked(value);
        }

        void* data() const
        {
            return m_buffer;
        }

        int size() const
        {
            return m_size;
        }

        void* executableCopy(ExecutablePool* allocator)
        {
            if (!m_size)
                return 0;

            void* result = allocator->alloc(m_size);

            if (!result)
                return 0;

            ExecutableAllocator::makeWritable(result, m_size);

            return memcpy(result, m_buffer, m_size);
        }

    protected:
        void append(const char* data, int size)
        {
            if (m_size > m_capacity - size)
                grow(size);

            memcpy(m_buffer + m_size, data, size);
            m_size += size;
        }

        void grow(int extraCapacity = 0)
        {
            m_capacity += m_capacity / 2 + extraCapacity;

            if (m_buffer == m_inlineBuffer) {
                char* newBuffer = static_cast<char*>(fastMalloc(m_capacity));
                m_buffer = static_cast<char*>(memcpy(newBuffer, m_buffer, m_size));
            } else
                m_buffer = static_cast<char*>(fastRealloc(m_buffer, m_capacity));
        }

        char m_inlineBuffer[inlineCapacity];
        char* m_buffer;
        int m_capacity;
        int m_size;
    };

} // namespace JSC

#endif // ENABLE(ASSEMBLER)

#endif // AssemblerBuffer_h
