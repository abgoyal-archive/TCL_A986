

#ifndef ResourceErrorBase_h
#define ResourceErrorBase_h

#include "PlatformString.h"

namespace WebCore {

class ResourceError;

class ResourceErrorBase {
public:
    // Makes a deep copy. Useful for when you need to use a ResourceError on another thread.
    ResourceError copy() const;

    bool isNull() const { return m_isNull; }

    const String& domain() const { lazyInit(); return m_domain; }
    int errorCode() const { lazyInit(); return m_errorCode; }
    const String& failingURL() const { lazyInit(); return m_failingURL; }
    const String& localizedDescription() const { lazyInit(); return m_localizedDescription; }

    void setIsCancellation(bool isCancellation) { m_isCancellation = isCancellation; }
    bool isCancellation() const { return m_isCancellation; }

    static bool compare(const ResourceError&, const ResourceError&);

protected:
    ResourceErrorBase()
        : m_errorCode(0)
        , m_isNull(true)
        , m_isCancellation(false)
    {
    }

    ResourceErrorBase(const String& domain, int errorCode, const String& failingURL, const String& localizedDescription)
        : m_domain(domain)
        , m_errorCode(errorCode)
        , m_failingURL(failingURL)
        , m_localizedDescription(localizedDescription)
        , m_isNull(false)
        , m_isCancellation(false)
    {
    }

    void lazyInit() const;

    // The ResourceError subclass may "shadow" this method to lazily initialize platform specific fields
    void platformLazyInit() {}

    // The ResourceError subclass may "shadow" this method to compare platform specific fields
    static bool platformCompare(const ResourceError&, const ResourceError&) { return true; }

    String m_domain;
    int m_errorCode;
    String m_failingURL;
    String m_localizedDescription;
    bool m_isNull;
    bool m_isCancellation;
};

inline bool operator==(const ResourceError& a, const ResourceError& b) { return ResourceErrorBase::compare(a, b); }
inline bool operator!=(const ResourceError& a, const ResourceError& b) { return !(a == b); }

} // namespace WebCore

#endif // ResourceErrorBase_h
