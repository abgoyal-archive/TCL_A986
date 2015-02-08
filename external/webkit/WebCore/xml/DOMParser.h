

#ifndef DOMParser_h
#define DOMParser_h

#include <wtf/RefCounted.h>
#include "Document.h"

namespace WebCore {

    class String;
    
    class DOMParser : public RefCounted<DOMParser> {
    public:
        static PassRefPtr<DOMParser> create() { return adoptRef(new DOMParser); }
        
        PassRefPtr<Document> parseFromString(const String& str, const String& contentType);
        
    private:
        DOMParser() { }        
    };

}

#endif // XMLSerializer.h
