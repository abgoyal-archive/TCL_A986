

#ifndef EventLoop_h
#define EventLoop_h

#include <wtf/Noncopyable.h>

namespace WebCore {

    class EventLoop : public Noncopyable {
    public:
        EventLoop()
            : m_ended(false)
        {
        }

        void cycle();
        bool ended() const { return m_ended; }

    private:
        bool m_ended;
    };

} // namespace WebCore

#endif // EventLoop_h
