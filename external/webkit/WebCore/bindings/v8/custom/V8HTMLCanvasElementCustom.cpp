

#include "config.h"
#include "V8HTMLCanvasElement.h"

#include "CanvasContextAttributes.h"
#include "CanvasRenderingContext.h"
#include "HTMLCanvasElement.h"
#include "WebGLContextAttributes.h"
#include "V8Binding.h"
#include "V8CanvasRenderingContext2D.h"
#include "V8Node.h"
#include "V8Proxy.h"
#include "V8WebGLRenderingContext.h"

namespace WebCore {

v8::Handle<v8::Value> V8HTMLCanvasElement::getContextCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.HTMLCanvasElement.context");
    v8::Handle<v8::Object> holder = args.Holder();
    HTMLCanvasElement* imp = V8HTMLCanvasElement::toNative(holder);
    String contextId = toWebCoreString(args[0]);
    RefPtr<CanvasContextAttributes> attrs;
#if ENABLE(3D_CANVAS)
    if (contextId == "experimental-webgl" || contextId == "webkit-3d") {
        attrs = WebGLContextAttributes::create();
        WebGLContextAttributes* webGLAttrs = static_cast<WebGLContextAttributes*>(attrs.get());
        if (args.Length() > 1 && args[0]->IsObject()) {
            v8::Handle<v8::Object> jsAttrs = args[1]->ToObject();
            v8::Handle<v8::String> alpha = v8::String::New("alpha");
            if (jsAttrs->Has(alpha))
                webGLAttrs->setAlpha(jsAttrs->Get(alpha)->BooleanValue());
            v8::Handle<v8::String> depth = v8::String::New("depth");
            if (jsAttrs->Has(depth))
                webGLAttrs->setDepth(jsAttrs->Get(depth)->BooleanValue());
            v8::Handle<v8::String> stencil = v8::String::New("stencil");
            if (jsAttrs->Has(stencil))
                webGLAttrs->setStencil(jsAttrs->Get(stencil)->BooleanValue());
            v8::Handle<v8::String> antialias = v8::String::New("antialias");
            if (jsAttrs->Has(antialias))
                webGLAttrs->setAntialias(jsAttrs->Get(antialias)->BooleanValue());
            v8::Handle<v8::String> premultipliedAlpha = v8::String::New("premultipliedAlpha");
            if (jsAttrs->Has(premultipliedAlpha))
                webGLAttrs->setPremultipliedAlpha(jsAttrs->Get(premultipliedAlpha)->BooleanValue());
        }
    }
#endif
    CanvasRenderingContext* result = imp->getContext(contextId, attrs.get());
    if (!result)
        return v8::Undefined();
    if (result->is2d())
        return toV8(static_cast<CanvasRenderingContext2D*>(result));
#if ENABLE(3D_CANVAS)
    else if (result->is3d())
        return toV8(static_cast<WebGLRenderingContext*>(result));
#endif
    ASSERT_NOT_REACHED();
    return v8::Undefined();
}

} // namespace WebCore
