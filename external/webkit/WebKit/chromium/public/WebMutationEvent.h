
#ifndef WebMutationEvent_h
#define WebMutationEvent_h

#include "WebEvent.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Event; }
#endif

namespace WebKit {

class WebMutationEvent : public WebEvent {
public:
    enum AttrChangeType {
        Modification    = 1,
        Addition        = 2,
        Removal         = 3
    };

    WEBKIT_API WebNode relatedNode() const;
    WEBKIT_API WebString prevValue() const;
    WEBKIT_API WebString newValue() const;
    WEBKIT_API WebString attrName() const;
    WEBKIT_API AttrChangeType attrChange() const;
};

} // namespace WebKit

#endif
