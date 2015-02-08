

#include "config.h"
#include "JSHTMLCanvasElement.h"

#include "CanvasContextAttributes.h"
#include "HTMLCanvasElement.h"
#include "JSCanvasRenderingContext2D.h"
#if ENABLE(3D_CANVAS)
#include "JSWebGLRenderingContext.h"
#include "WebGLContextAttributes.h"
#endif
#include <wtf/GetPtr.h>

using namespace JSC;

namespace WebCore {

void JSHTMLCanvasElement::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    HTMLCanvasElement* canvas = static_cast<HTMLCanvasElement*>(impl());
    JSGlobalData& globalData = *Heap::heap(this)->globalData();

    markDOMObjectWrapper(markStack, globalData, canvas->renderingContext());
}

JSValue JSHTMLCanvasElement::getContext(ExecState* exec, const ArgList& args)
{
    HTMLCanvasElement* canvas = static_cast<HTMLCanvasElement*>(impl());
    const UString& contextId = args.at(0).toString(exec);
    RefPtr<CanvasContextAttributes> attrs;
#if ENABLE(3D_CANVAS)
    if (contextId == "experimental-webgl" || contextId == "webkit-3d") {
        attrs = WebGLContextAttributes::create();
        WebGLContextAttributes* webGLAttrs = static_cast<WebGLContextAttributes*>(attrs.get());
        if (args.size() > 1 && args.at(1).isObject()) {
            JSObject* jsAttrs = args.at(1).getObject();
            Identifier alpha(exec, "alpha");
            if (jsAttrs->hasProperty(exec, alpha))
                webGLAttrs->setAlpha(jsAttrs->get(exec, alpha).toBoolean(exec));
            Identifier depth(exec, "depth");
            if (jsAttrs->hasProperty(exec, depth))
                webGLAttrs->setDepth(jsAttrs->get(exec, depth).toBoolean(exec));
            Identifier stencil(exec, "stencil");
            if (jsAttrs->hasProperty(exec, stencil))
                webGLAttrs->setStencil(jsAttrs->get(exec, stencil).toBoolean(exec));
            Identifier antialias(exec, "antialias");
            if (jsAttrs->hasProperty(exec, antialias))
                webGLAttrs->setAntialias(jsAttrs->get(exec, antialias).toBoolean(exec));
            Identifier premultipliedAlpha(exec, "premultipliedAlpha");
            if (jsAttrs->hasProperty(exec, premultipliedAlpha))
                webGLAttrs->setPremultipliedAlpha(jsAttrs->get(exec, premultipliedAlpha).toBoolean(exec));
        }
    }
#endif
    return toJS(exec, globalObject(), WTF::getPtr(canvas->getContext(contextId, attrs.get())));
}

} // namespace WebCore
