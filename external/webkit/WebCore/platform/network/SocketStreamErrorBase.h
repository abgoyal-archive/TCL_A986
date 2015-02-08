

#ifndef SocketStreamErrorBase_h
#define SocketStreamErrorBase_h

namespace WebCore {

    class SocketStreamError;

    class SocketStreamErrorBase {
    public:
        // Makes a deep copy.  Useful for when you need to use a SocketStreamError on another thread.
        SocketStreamError copy() const;

        bool isNull() const { return m_isNull; }

        int errorCode() const { return m_errorCode; }

        static bool compare(const SocketStreamError&, const SocketStreamError&);

    protected:
        SocketStreamErrorBase()
            : m_errorCode(0)
            , m_isNull(true)
        {
        }

        explicit SocketStreamErrorBase(int errorCode)
            : m_errorCode(errorCode)
            , m_isNull(false)
        {
        }

        int m_errorCode;
        bool m_isNull;
    };

    inline bool operator==(const SocketStreamError& a, const SocketStreamError& b) { return SocketStreamErrorBase::compare(a, b); }
    inline bool operator!=(const SocketStreamError& a, const SocketStreamError& b) { return !(a == b); }

}  // namespace WebCore

#endif  // SocketStreamErrorBase_h
