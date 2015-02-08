

#include "config.h"
#include "JSDOMApplicationCache.h"

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "AtomicString.h"
#include "DOMApplicationCache.h"
#include "DOMWindow.h"
#include "Event.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "JSDOMWindowCustom.h"
#include "JSEvent.h"
#include "JSEventListener.h"

using namespace JSC;

namespace WebCore {

#if ENABLE(APPLICATION_CACHE_DYNAMIC_ENTRIES)

JSValue JSDOMApplicationCache::hasItem(ExecState* exec, const ArgList& args)
{
    Frame* frame = asJSDOMWindow(exec->dynamicGlobalObject())->impl()->frame();
    if (!frame)
        return jsUndefined();
    const KURL& url = frame->loader()->completeURL(args.at(0).toString(exec));

    ExceptionCode ec = 0;
    bool result = impl()->hasItem(url, ec);
    setDOMException(exec, ec);
    return jsBoolean(result);
}

JSValue JSDOMApplicationCache::add(ExecState* exec, const ArgList& args)
{
    Frame* frame = asJSDOMWindow(exec->dynamicGlobalObject())->impl()->frame();
    if (!frame)
        return jsUndefined();
    const KURL& url = frame->loader()->completeURL(args.at(0).toString(exec));
    
    ExceptionCode ec = 0;
    impl()->add(url, ec);
    setDOMException(exec, ec);
    return jsUndefined();
}

JSValue JSDOMApplicationCache::remove(ExecState* exec, const ArgList& args)
{
    Frame* frame = asJSDOMWindow(exec->dynamicGlobalObject())->impl()->frame();
    if (!frame)
        return jsUndefined();
    const KURL& url = frame->loader()->completeURL(args.at(0).toString(exec));
    
    ExceptionCode ec = 0;
    impl()->remove(url, ec);
    setDOMException(exec, ec);
    return jsUndefined();
}

#endif // ENABLE(APPLICATION_CACHE_DYNAMIC_ENTRIES)

JSValue JSDOMApplicationCache::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSDOMApplicationCache::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

} // namespace WebCore

#endif // ENABLE(OFFLINE_WEB_APPLICATIONS)
