

#ifndef PositionOptions_h
#define PositionOptions_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
class PositionOptions : public RefCounted<PositionOptions> {
public:
    static PassRefPtr<PositionOptions> create() { return adoptRef(new PositionOptions()); }

    bool enableHighAccuracy() const { return m_highAccuracy; }
    void setEnableHighAccuracy(bool enable) { m_highAccuracy = enable; }
    bool hasTimeout() const { return m_hasTimeout; }
    int timeout() const
    {
        ASSERT(hasTimeout());
        return m_timeout;
    }
    void setTimeout(int timeout)
    {
        ASSERT(timeout >= 0);
        m_hasTimeout = true;
        m_timeout = timeout;
    }
    bool hasMaximumAge() const { return m_hasMaximumAge; }
    int maximumAge() const
    {
        ASSERT(hasMaximumAge());
        return m_maximumAge;
    }
    void clearMaximumAge() { m_hasMaximumAge = false; }
    void setMaximumAge(int age)
    {
        ASSERT(age >= 0);
        m_hasMaximumAge = true;
        m_maximumAge = age;
    }
    
private:
    PositionOptions()
        : m_highAccuracy(false)
        , m_hasTimeout(false)
    {
        setMaximumAge(0);
    }
    
    bool m_highAccuracy;
    bool m_hasTimeout;
    int m_timeout;
    bool m_hasMaximumAge;
    int m_maximumAge;
};
    
} // namespace WebCore

#endif // PositionOptions_h
