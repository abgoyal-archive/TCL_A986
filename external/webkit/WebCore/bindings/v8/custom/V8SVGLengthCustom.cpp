
 
#include <config.h>

#if ENABLE(SVG)
#include "V8SVGLength.h"

#include "SVGLength.h"
#include "V8Binding.h"
#include "V8SVGPODTypeWrapper.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8SVGLength::valueAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.SVGLength.value");
    V8SVGPODTypeWrapper<SVGLength>* wrapper = V8SVGPODTypeWrapper<SVGLength>::toNative(info.Holder());
    SVGLength imp = *wrapper;
    return v8::Number::New(imp.value(V8Proxy::svgContext(wrapper)));
}

v8::Handle<v8::Value> V8SVGLength::convertToSpecifiedUnitsCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.SVGLength.convertToSpecifiedUnits");
    V8SVGPODTypeWrapper<SVGLength>* wrapper = V8SVGPODTypeWrapper<SVGLength>::toNative(args.Holder());
    SVGLength imp = *wrapper;
    SVGElement* context = V8Proxy::svgContext(wrapper);
    imp.convertToSpecifiedUnits(toInt32(args[0]), context);
    wrapper->commitChange(imp, context);
    return v8::Undefined();
}

} // namespace WebCore

#endif
