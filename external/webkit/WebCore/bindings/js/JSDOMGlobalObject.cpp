

#include "config.h"
#include "JSDOMGlobalObject.h"

#include "Document.h"
#include "JSDOMWindow.h"
#include "JSEventListener.h"

#if ENABLE(WORKERS)
#include "JSWorkerContext.h"
#include "WorkerContext.h"
#endif

using namespace JSC;

namespace WebCore {

const ClassInfo JSDOMGlobalObject::s_info = { "DOMGlobalObject", 0, 0, 0 };

JSDOMGlobalObject::JSDOMGlobalObject(NonNullPassRefPtr<Structure> structure, JSDOMGlobalObject::JSDOMGlobalObjectData* data, JSObject* thisValue)
    : JSGlobalObject(structure, data, thisValue)
{
}

void JSDOMGlobalObject::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    JSDOMStructureMap::iterator end = structures().end();
    for (JSDOMStructureMap::iterator it = structures().begin(); it != end; ++it)
        markStack.append(it->second->storedPrototype());

    JSDOMConstructorMap::iterator end2 = constructors().end();
    for (JSDOMConstructorMap::iterator it2 = constructors().begin(); it2 != end2; ++it2)
        markStack.append(it2->second);

    if (d()->m_injectedScript)
        markStack.append(d()->m_injectedScript);
}

void JSDOMGlobalObject::setCurrentEvent(Event* evt)
{
    d()->evt = evt;
}

Event* JSDOMGlobalObject::currentEvent() const
{
    return d()->evt;
}

void JSDOMGlobalObject::setInjectedScript(JSObject* injectedScript)
{
    d()->m_injectedScript = injectedScript;
}

JSObject* JSDOMGlobalObject::injectedScript() const
{
    return d()->m_injectedScript;
}

void JSDOMGlobalObject::destroyJSDOMGlobalObjectData(void* jsDOMGlobalObjectData)
{
    delete static_cast<JSDOMGlobalObjectData*>(jsDOMGlobalObjectData);
}

JSDOMGlobalObject* toJSDOMGlobalObject(Document* document, JSC::ExecState* exec)
{
    return toJSDOMWindow(document->frame(), currentWorld(exec));
}

JSDOMGlobalObject* toJSDOMGlobalObject(ScriptExecutionContext* scriptExecutionContext, JSC::ExecState* exec)
{
    if (scriptExecutionContext->isDocument())
        return toJSDOMGlobalObject(static_cast<Document*>(scriptExecutionContext), exec);

#if ENABLE(WORKERS)
    if (scriptExecutionContext->isWorkerContext())
        return static_cast<WorkerContext*>(scriptExecutionContext)->script()->workerContextWrapper();
#endif

    ASSERT_NOT_REACHED();
    return 0;
}

JSDOMGlobalObject* toJSDOMGlobalObject(Document* document, DOMWrapperWorld* world)
{
    return toJSDOMWindow(document->frame(), world);
}

JSDOMGlobalObject* toJSDOMGlobalObject(ScriptExecutionContext* scriptExecutionContext, DOMWrapperWorld* world)
{
    if (scriptExecutionContext->isDocument())
        return toJSDOMGlobalObject(static_cast<Document*>(scriptExecutionContext), world);

#if ENABLE(WORKERS)
    if (scriptExecutionContext->isWorkerContext())
        return static_cast<WorkerContext*>(scriptExecutionContext)->script()->workerContextWrapper();
#endif

    ASSERT_NOT_REACHED();
    return 0;
}

} // namespace WebCore
