

#include "config.h"
#include "WebMutationEvent.h"

#include "MutationEvent.h"

using namespace WebCore;

namespace WebKit {

WebNode WebMutationEvent::relatedNode() const
{
    return WebNode(constUnwrap<MutationEvent>()->relatedNode());
}

WebString WebMutationEvent::prevValue() const
{
    return WebString(constUnwrap<MutationEvent>()->prevValue());
}

WebString WebMutationEvent::newValue() const
{
    return WebString(constUnwrap<MutationEvent>()->newValue());
}

WebString WebMutationEvent::attrName() const
{
    return WebString(constUnwrap<MutationEvent>()->attrName());
}

WebMutationEvent::AttrChangeType WebMutationEvent::attrChange() const
{
    return static_cast<AttrChangeType>(constUnwrap<MutationEvent>()->attrChange());
}

} // namespace WebKit
