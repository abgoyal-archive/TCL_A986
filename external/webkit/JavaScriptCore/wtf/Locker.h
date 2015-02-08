
#ifndef Locker_h
#define Locker_h

#include <wtf/Noncopyable.h>

namespace WTF {

template <typename T> class Locker : public Noncopyable {
public:
    Locker(T& lockable) : m_lockable(lockable) { m_lockable.lock(); }
    ~Locker() { m_lockable.unlock(); }
private:
    T& m_lockable;
};

}

using WTF::Locker;

#endif
