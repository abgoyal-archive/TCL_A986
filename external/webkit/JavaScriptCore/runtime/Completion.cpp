

#include "config.h"
#include "Completion.h"

#include "CallFrame.h"
#include "JSGlobalObject.h"
#include "JSLock.h"
#include "Interpreter.h"
#include "Parser.h"
#include "Debugger.h"
#include <stdio.h>

namespace JSC {

Completion checkSyntax(ExecState* exec, const SourceCode& source)
{
    JSLock lock(exec);
    ASSERT(exec->globalData().identifierTable == currentIdentifierTable());

    RefPtr<ProgramExecutable> program = ProgramExecutable::create(exec, source);
    JSObject* error = program->checkSyntax(exec);
    if (error)
        return Completion(Throw, error);

    return Completion(Normal);
}

Completion evaluate(ExecState* exec, ScopeChain& scopeChain, const SourceCode& source, JSValue thisValue)
{
    JSLock lock(exec);
    ASSERT(exec->globalData().identifierTable == currentIdentifierTable());

    RefPtr<ProgramExecutable> program = ProgramExecutable::create(exec, source);
    JSObject* error = program->compile(exec, scopeChain.node());
    if (error)
        return Completion(Throw, error);

    JSObject* thisObj = (!thisValue || thisValue.isUndefinedOrNull()) ? exec->dynamicGlobalObject() : thisValue.toObject(exec);

    JSValue exception;
    JSValue result = exec->interpreter()->execute(program.get(), exec, scopeChain.node(), thisObj, &exception);

    if (exception) {
        if (exception.isObject() && asObject(exception)->isWatchdogException())
            return Completion(Interrupted, exception);
        return Completion(Throw, exception);
    }
    return Completion(Normal, result);
}

} // namespace JSC
