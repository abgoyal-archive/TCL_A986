

#include "config.h"
#include "JSMessageChannelConstructor.h"

#include "Document.h"
#include "JSDocument.h"
#include "JSMessageChannel.h"
#include "MessageChannel.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

const ClassInfo JSMessageChannelConstructor::s_info = { "MessageChannelConstructor", 0, 0, 0 };

JSMessageChannelConstructor::JSMessageChannelConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSMessageChannelConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSMessageChannelPrototype::self(exec, globalObject), None);
}

JSMessageChannelConstructor::~JSMessageChannelConstructor()
{
}

ConstructType JSMessageChannelConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = construct;
    return ConstructTypeHost;
}

JSObject* JSMessageChannelConstructor::construct(ExecState* exec, JSObject* constructor, const ArgList&)
{
    JSMessageChannelConstructor* jsConstructor = static_cast<JSMessageChannelConstructor*>(constructor);
    ScriptExecutionContext* context = jsConstructor->scriptExecutionContext();
    if (!context)
        return throwError(exec, ReferenceError, "MessageChannel constructor associated document is unavailable");

    return asObject(toJS(exec, jsConstructor->globalObject(), MessageChannel::create(context)));
}

} // namespace WebCore
