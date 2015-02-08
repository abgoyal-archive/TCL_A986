

#ifndef XMLSerializer_h
#define XMLSerializer_h

#include <wtf/RefCounted.h>
#include "PlatformString.h"

namespace WebCore {

    typedef int ExceptionCode;

    class Node;

    class XMLSerializer : public RefCounted<XMLSerializer> {
    public:
        static PassRefPtr<XMLSerializer> create() { return adoptRef(new XMLSerializer); }
        
        String serializeToString(Node*, ExceptionCode&);
        
    private:
        XMLSerializer()  { }        
    };

} // namespace WebCore

#endif // XMLSerializer_h
