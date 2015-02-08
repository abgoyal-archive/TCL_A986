

#include "config.h"

#if ENABLE(SVG)
#include "JSSVGElementInstance.h"

#include "JSDOMWindow.h"
#include "JSEventListener.h"
#include "JSSVGElement.h"
#include "SVGElementInstance.h"

using namespace JSC;

namespace WebCore {

void JSSVGElementInstance::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    // Mark the wrapper for our corresponding element, so it can mark its event handlers.
    markDOMNodeWrapper(markStack, impl()->correspondingElement()->document(), impl()->correspondingElement());
}

JSValue JSSVGElementInstance::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSSVGElementInstance::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

void JSSVGElementInstance::pushEventHandlerScope(ExecState*, ScopeChain&) const
{
}

JSC::JSValue toJS(JSC::ExecState* exec, JSDOMGlobalObject* globalObject, SVGElementInstance* object)
{
    JSValue result = getDOMObjectWrapper<JSSVGElementInstance>(exec, globalObject, object);

    // Ensure that our corresponding element has a JavaScript wrapper to keep its event handlers alive.
    if (object)
        toJS(exec, object->correspondingElement());

    return result;
}

} // namespace WebCore

#endif // ENABLE(SVG)
