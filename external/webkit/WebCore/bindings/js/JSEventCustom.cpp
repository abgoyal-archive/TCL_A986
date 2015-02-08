

#include "config.h"
#include "JSEvent.h"

#include "Clipboard.h"
#include "CompositionEvent.h"
#include "Event.h"
#include "JSBeforeLoadEvent.h"
#include "JSClipboard.h"
#include "JSCompositionEvent.h"
#include "JSErrorEvent.h"
#include "JSKeyboardEvent.h"
#include "JSMessageEvent.h"
#include "JSMouseEvent.h"
#include "JSMutationEvent.h"
#include "JSOverflowEvent.h"
#include "JSPageTransitionEvent.h"
#include "JSPopStateEvent.h"
#include "JSProgressEvent.h"
#include "JSTextEvent.h"
#include "JSUIEvent.h"
#include "JSWebKitAnimationEvent.h"
#include "JSWebKitTransitionEvent.h"
#include "JSWheelEvent.h"
#include "JSXMLHttpRequestProgressEvent.h"
#include "BeforeLoadEvent.h"
#include "ErrorEvent.h"
#include "KeyboardEvent.h"
#include "MessageEvent.h"
#include "MouseEvent.h"
#include "MutationEvent.h"
#include "OverflowEvent.h"
#include "PageTransitionEvent.h"
#include "PopStateEvent.h"
#include "ProgressEvent.h"
#include "TextEvent.h"
#include "UIEvent.h"
#include "WebKitAnimationEvent.h"
#include "WebKitTransitionEvent.h"
#include "WheelEvent.h"
#include "XMLHttpRequestProgressEvent.h"
#include <runtime/JSLock.h>

#if ENABLE(DOM_STORAGE)
#include "JSStorageEvent.h"
#include "StorageEvent.h"
#endif

#if ENABLE(SVG)
#include "JSSVGZoomEvent.h"
#include "SVGZoomEvent.h"
#endif

#if ENABLE(TOUCH_EVENTS)
#include "JSTouchEvent.h"
#include "TouchEvent.h"
#endif

using namespace JSC;

namespace WebCore {

JSValue JSEvent::clipboardData(ExecState* exec) const
{
    return impl()->isClipboardEvent() ? toJS(exec, globalObject(), impl()->clipboardData()) : jsUndefined();
}

JSValue toJS(ExecState* exec, JSDOMGlobalObject* globalObject, Event* event)
{
    JSLock lock(SilenceAssertionsOnly);

    if (!event)
        return jsNull();

    DOMObject* wrapper = getCachedDOMObjectWrapper(exec, event);
    if (wrapper)
        return wrapper;

    if (event->isUIEvent()) {
        if (event->isKeyboardEvent())
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, KeyboardEvent, event);
        else if (event->isTextEvent())
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, TextEvent, event);
        else if (event->isMouseEvent())
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, MouseEvent, event);
        else if (event->isWheelEvent())
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, WheelEvent, event);
#if ENABLE(SVG)
        else if (event->isSVGZoomEvent())
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, SVGZoomEvent, event);
#endif
        else if (event->isCompositionEvent())
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CompositionEvent, event);
#if ENABLE(TOUCH_EVENTS)
        else if (event->isTouchEvent())
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, TouchEvent, event);
#endif
        else
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, UIEvent, event);
    } else if (event->isMutationEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, MutationEvent, event);
    else if (event->isOverflowEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, OverflowEvent, event);
    else if (event->isMessageEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, MessageEvent, event);
    else if (event->isPageTransitionEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, PageTransitionEvent, event);
    else if (event->isProgressEvent()) {
        if (event->isXMLHttpRequestProgressEvent())
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, XMLHttpRequestProgressEvent, event);
        else
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, ProgressEvent, event);
    } else if (event->isBeforeLoadEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, BeforeLoadEvent, event);
#if ENABLE(DOM_STORAGE)
    else if (event->isStorageEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, StorageEvent, event);
#endif
    else if (event->isWebKitAnimationEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, WebKitAnimationEvent, event);
    else if (event->isWebKitTransitionEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, WebKitTransitionEvent, event);
#if ENABLE(WORKERS)
    else if (event->isErrorEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, ErrorEvent, event);
#endif
    else if (event->isPopStateEvent())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, PopStateEvent, event);
    else
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, Event, event);

    return wrapper;
}

} // namespace WebCore
