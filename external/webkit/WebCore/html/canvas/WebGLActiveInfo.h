

#ifndef WebGLActiveInfo_h
#define WebGLActiveInfo_h

#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLActiveInfo : public RefCounted<WebGLActiveInfo> {
public:
    static PassRefPtr<WebGLActiveInfo> create(const String& name, unsigned type, int size)
    {
        return adoptRef(new WebGLActiveInfo(name, type, size));
    }
    String name() const { return m_name; }
    unsigned type() const { return m_type; }
    int size() const { return m_size; }

private:
    WebGLActiveInfo(const String& name, unsigned type, int size)
        : m_name(name)
        , m_type(type)
        , m_size(size)
    {
        ASSERT(name.length());
        ASSERT(type);
        ASSERT(size);
    }
    String m_name;
    unsigned m_type;
    int m_size;
};

}

#endif // WebGLActiveInfo_h
