

#include "config.h"
#include "V8HTMLElement.h"

#include "V8HTMLElementWrapperFactory.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(HTMLElement* impl, bool forceNewObject)
{
    if (!impl)
        return v8::Null();
    return createV8HTMLWrapper(impl, forceNewObject);
}

} // namespace WebCore
