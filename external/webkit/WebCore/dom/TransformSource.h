

#ifndef TransformSource_h
#define TransformSource_h

#if ENABLE(XSLT)

#include "PlatformString.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

#if USE(QXMLQUERY)
    typedef String PlatformTransformSource;
#else
    typedef void* PlatformTransformSource;
#endif

    class TransformSource : public Noncopyable {
    public:
        TransformSource(const PlatformTransformSource& source);
        ~TransformSource();

        PlatformTransformSource platformSource() const { return m_source; }

    private:
        PlatformTransformSource m_source;
    };

} // namespace WebCore

#endif

#endif // TransformSource_h
