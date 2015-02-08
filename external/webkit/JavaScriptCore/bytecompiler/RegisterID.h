

#ifndef RegisterID_h
#define RegisterID_h

#include <wtf/Assertions.h>
#include <wtf/Noncopyable.h>
#include <wtf/VectorTraits.h>

namespace JSC {

    class RegisterID : public Noncopyable {
    public:
        RegisterID()
            : m_refCount(0)
            , m_isTemporary(false)
#ifndef NDEBUG
            , m_didSetIndex(false)
#endif
        {
        }

        explicit RegisterID(int index)
            : m_refCount(0)
            , m_index(index)
            , m_isTemporary(false)
#ifndef NDEBUG
            , m_didSetIndex(true)
#endif
        {
        }

        void setIndex(int index)
        {
            ASSERT(!m_refCount);
#ifndef NDEBUG
            m_didSetIndex = true;
#endif
            m_index = index;
        }

        void setTemporary()
        {
            m_isTemporary = true;
        }

        int index() const
        {
            ASSERT(m_didSetIndex);
            return m_index;
        }

        bool isTemporary()
        {
            return m_isTemporary;
        }

        void ref()
        {
            ++m_refCount;
        }

        void deref()
        {
            --m_refCount;
            ASSERT(m_refCount >= 0);
        }

        int refCount() const
        {
            return m_refCount;
        }

    private:

        int m_refCount;
        int m_index;
        bool m_isTemporary;
#ifndef NDEBUG
        bool m_didSetIndex;
#endif
    };

} // namespace JSC

namespace WTF {

    template<> struct VectorTraits<JSC::RegisterID> : VectorTraitsBase<true, JSC::RegisterID> {
        static const bool needsInitialization = true;
        static const bool canInitializeWithMemset = true; // Default initialization just sets everything to 0 or false, so this is safe.
    };

} // namespace WTF

#endif // RegisterID_h
