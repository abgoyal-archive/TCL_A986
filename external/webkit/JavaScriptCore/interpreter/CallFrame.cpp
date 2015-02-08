

#include "config.h"
#include "CallFrame.h"

#include "CodeBlock.h"
#include "Interpreter.h"

namespace JSC {

JSValue CallFrame::thisValue()
{
    return this[codeBlock()->thisRegister()].jsValue();
}

#ifndef NDEBUG
void CallFrame::dumpCaller()
{
    int signedLineNumber;
    intptr_t sourceID;
    UString urlString;
    JSValue function;
    
    interpreter()->retrieveLastCaller(this, signedLineNumber, sourceID, urlString, function);
    printf("Callpoint => %s:%d\n", urlString.ascii(), signedLineNumber);
}
#endif

}
