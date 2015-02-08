

#ifndef APICast_h
#define APICast_h

#include "JSAPIValueWrapper.h"
#include "JSGlobalObject.h"
#include "JSValue.h"
#include <wtf/Platform.h>
#include <wtf/UnusedParam.h>

namespace JSC {
    class ExecState;
    class PropertyNameArray;
    class JSGlobalData;
    class JSObject;
    class JSValue;
}

typedef const struct OpaqueJSContextGroup* JSContextGroupRef;
typedef const struct OpaqueJSContext* JSContextRef;
typedef struct OpaqueJSContext* JSGlobalContextRef;
typedef struct OpaqueJSPropertyNameAccumulator* JSPropertyNameAccumulatorRef;
typedef const struct OpaqueJSValue* JSValueRef;
typedef struct OpaqueJSValue* JSObjectRef;

/* Opaque typing convenience methods */

inline JSC::ExecState* toJS(JSContextRef c)
{
    ASSERT(c);
    return reinterpret_cast<JSC::ExecState*>(const_cast<OpaqueJSContext*>(c));
}

inline JSC::ExecState* toJS(JSGlobalContextRef c)
{
    ASSERT(c);
    return reinterpret_cast<JSC::ExecState*>(c);
}

inline JSC::JSValue toJS(JSC::ExecState* exec, JSValueRef v)
{
    ASSERT_UNUSED(exec, exec);
    ASSERT(v);
#if USE(JSVALUE32_64)
    JSC::JSCell* jsCell = reinterpret_cast<JSC::JSCell*>(const_cast<OpaqueJSValue*>(v));
    if (!jsCell)
        return JSC::JSValue();
    if (jsCell->isAPIValueWrapper())
        return static_cast<JSC::JSAPIValueWrapper*>(jsCell)->value();
    return jsCell;
#else
    return JSC::JSValue::decode(reinterpret_cast<JSC::EncodedJSValue>(const_cast<OpaqueJSValue*>(v)));
#endif
}

inline JSC::JSValue toJSForGC(JSC::ExecState* exec, JSValueRef v)
{
    ASSERT_UNUSED(exec, exec);
    ASSERT(v);
#if USE(JSVALUE32_64)
    JSC::JSCell* jsCell = reinterpret_cast<JSC::JSCell*>(const_cast<OpaqueJSValue*>(v));
    if (!jsCell)
        return JSC::JSValue();
    return jsCell;
#else
    return JSC::JSValue::decode(reinterpret_cast<JSC::EncodedJSValue>(const_cast<OpaqueJSValue*>(v)));
#endif
}

inline JSC::JSObject* toJS(JSObjectRef o)
{
    return reinterpret_cast<JSC::JSObject*>(o);
}

inline JSC::PropertyNameArray* toJS(JSPropertyNameAccumulatorRef a)
{
    return reinterpret_cast<JSC::PropertyNameArray*>(a);
}

inline JSC::JSGlobalData* toJS(JSContextGroupRef g)
{
    return reinterpret_cast<JSC::JSGlobalData*>(const_cast<OpaqueJSContextGroup*>(g));
}

inline JSValueRef toRef(JSC::ExecState* exec, JSC::JSValue v)
{
#if USE(JSVALUE32_64)
    if (!v)
        return 0;
    if (!v.isCell())
        return reinterpret_cast<JSValueRef>(asCell(JSC::jsAPIValueWrapper(exec, v)));
    return reinterpret_cast<JSValueRef>(asCell(v));
#else
    UNUSED_PARAM(exec);
    return reinterpret_cast<JSValueRef>(JSC::JSValue::encode(v));
#endif
}

inline JSObjectRef toRef(JSC::JSObject* o)
{
    return reinterpret_cast<JSObjectRef>(o);
}

inline JSObjectRef toRef(const JSC::JSObject* o)
{
    return reinterpret_cast<JSObjectRef>(const_cast<JSC::JSObject*>(o));
}

inline JSContextRef toRef(JSC::ExecState* e)
{
    return reinterpret_cast<JSContextRef>(e);
}

inline JSGlobalContextRef toGlobalRef(JSC::ExecState* e)
{
    ASSERT(e == e->lexicalGlobalObject()->globalExec());
    return reinterpret_cast<JSGlobalContextRef>(e);
}

inline JSPropertyNameAccumulatorRef toRef(JSC::PropertyNameArray* l)
{
    return reinterpret_cast<JSPropertyNameAccumulatorRef>(l);
}

inline JSContextGroupRef toRef(JSC::JSGlobalData* g)
{
    return reinterpret_cast<JSContextGroupRef>(g);
}

#endif // APICast_h
