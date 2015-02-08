

#include "config.h"
#include "V8SVGElement.h"

#if ENABLE(SVG)

#include "V8SVGElementWrapperFactory.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(SVGElement* impl, bool forceNewObject)
{
    if (!impl)
        return v8::Null();
    return createV8SVGWrapper(impl, forceNewObject);
}

} // namespace WebCore

#endif // ENABLE(SVG)
