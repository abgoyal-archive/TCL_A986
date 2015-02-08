

#include "config.h"
#include "ExceptionHelpers.h"

#include "CodeBlock.h"
#include "CallFrame.h"
#include "JSGlobalObjectFunctions.h"
#include "JSObject.h"
#include "JSNotAnObject.h"
#include "Interpreter.h"
#include "Nodes.h"

namespace JSC {

class InterruptedExecutionError : public JSObject {
public:
    InterruptedExecutionError(JSGlobalData* globalData)
        : JSObject(globalData->interruptedExecutionErrorStructure)
    {
    }

    virtual bool isWatchdogException() const { return true; }

    virtual UString toString(ExecState*) const { return "JavaScript execution exceeded timeout."; }
};

JSValue createInterruptedExecutionException(JSGlobalData* globalData)
{
    return new (globalData) InterruptedExecutionError(globalData);
}

static JSValue createError(ExecState* exec, ErrorType e, const char* msg)
{
    return Error::create(exec, e, msg, -1, -1, UString());
}

JSValue createStackOverflowError(ExecState* exec)
{
    return createError(exec, RangeError, "Maximum call stack size exceeded.");
}

JSValue createTypeError(ExecState* exec, const char* message)
{
    return createError(exec, TypeError, message);
}

JSValue createUndefinedVariableError(ExecState* exec, const Identifier& ident, unsigned bytecodeOffset, CodeBlock* codeBlock)
{
    int startOffset = 0;
    int endOffset = 0;
    int divotPoint = 0;
    int line = codeBlock->expressionRangeForBytecodeOffset(exec, bytecodeOffset, divotPoint, startOffset, endOffset);
    JSObject* exception = Error::create(exec, ReferenceError, makeString("Can't find variable: ", ident.ustring()), line, codeBlock->ownerExecutable()->sourceID(), codeBlock->ownerExecutable()->sourceURL());
    exception->putWithAttributes(exec, Identifier(exec, expressionBeginOffsetPropertyName), jsNumber(exec, divotPoint - startOffset), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionCaretOffsetPropertyName), jsNumber(exec, divotPoint), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionEndOffsetPropertyName), jsNumber(exec, divotPoint + endOffset), ReadOnly | DontDelete);
    return exception;
}
    
static UString createErrorMessage(ExecState* exec, CodeBlock* codeBlock, int, int expressionStart, int expressionStop, JSValue value, UString error)
{
    if (!expressionStop || expressionStart > codeBlock->source()->length())
        return makeString(value.toString(exec), " is ", error);
    if (expressionStart < expressionStop)
        return makeString("Result of expression '", codeBlock->source()->getRange(expressionStart, expressionStop), "' [", value.toString(exec), "] is ", error, ".");

    // No range information, so give a few characters of context
    const UChar* data = codeBlock->source()->data();
    int dataLength = codeBlock->source()->length();
    int start = expressionStart;
    int stop = expressionStart;
    // Get up to 20 characters of context to the left and right of the divot, clamping to the line.
    // then strip whitespace.
    while (start > 0 && (expressionStart - start < 20) && data[start - 1] != '\n')
        start--;
    while (start < (expressionStart - 1) && isStrWhiteSpace(data[start]))
        start++;
    while (stop < dataLength && (stop - expressionStart < 20) && data[stop] != '\n')
        stop++;
    while (stop > expressionStart && isStrWhiteSpace(data[stop]))
        stop--;
    return makeString("Result of expression near '...", codeBlock->source()->getRange(start, stop), "...' [", value.toString(exec), "] is ", error, ".");
}

JSObject* createInvalidParamError(ExecState* exec, const char* op, JSValue value, unsigned bytecodeOffset, CodeBlock* codeBlock)
{
    int startOffset = 0;
    int endOffset = 0;
    int divotPoint = 0;
    int line = codeBlock->expressionRangeForBytecodeOffset(exec, bytecodeOffset, divotPoint, startOffset, endOffset);
    UString errorMessage = createErrorMessage(exec, codeBlock, line, divotPoint, divotPoint + endOffset, value, makeString("not a valid argument for '", op, "'"));
    JSObject* exception = Error::create(exec, TypeError, errorMessage, line, codeBlock->ownerExecutable()->sourceID(), codeBlock->ownerExecutable()->sourceURL());
    exception->putWithAttributes(exec, Identifier(exec, expressionBeginOffsetPropertyName), jsNumber(exec, divotPoint - startOffset), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionCaretOffsetPropertyName), jsNumber(exec, divotPoint), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionEndOffsetPropertyName), jsNumber(exec, divotPoint + endOffset), ReadOnly | DontDelete);
    return exception;
}

JSObject* createNotAConstructorError(ExecState* exec, JSValue value, unsigned bytecodeOffset, CodeBlock* codeBlock)
{
    int startOffset = 0;
    int endOffset = 0;
    int divotPoint = 0;
    int line = codeBlock->expressionRangeForBytecodeOffset(exec, bytecodeOffset, divotPoint, startOffset, endOffset);

    // We're in a "new" expression, so we need to skip over the "new.." part
    int startPoint = divotPoint - (startOffset ? startOffset - 4 : 0); // -4 for "new "
    const UChar* data = codeBlock->source()->data();
    while (startPoint < divotPoint && isStrWhiteSpace(data[startPoint]))
        startPoint++;
    
    UString errorMessage = createErrorMessage(exec, codeBlock, line, startPoint, divotPoint, value, "not a constructor");
    JSObject* exception = Error::create(exec, TypeError, errorMessage, line, codeBlock->ownerExecutable()->sourceID(), codeBlock->ownerExecutable()->sourceURL());
    exception->putWithAttributes(exec, Identifier(exec, expressionBeginOffsetPropertyName), jsNumber(exec, divotPoint - startOffset), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionCaretOffsetPropertyName), jsNumber(exec, divotPoint), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionEndOffsetPropertyName), jsNumber(exec, divotPoint + endOffset), ReadOnly | DontDelete);
    return exception;
}

JSValue createNotAFunctionError(ExecState* exec, JSValue value, unsigned bytecodeOffset, CodeBlock* codeBlock)
{
    int startOffset = 0;
    int endOffset = 0;
    int divotPoint = 0;
    int line = codeBlock->expressionRangeForBytecodeOffset(exec, bytecodeOffset, divotPoint, startOffset, endOffset);
    UString errorMessage = createErrorMessage(exec, codeBlock, line, divotPoint - startOffset, divotPoint, value, "not a function");
    JSObject* exception = Error::create(exec, TypeError, errorMessage, line, codeBlock->ownerExecutable()->sourceID(), codeBlock->ownerExecutable()->sourceURL());    
    exception->putWithAttributes(exec, Identifier(exec, expressionBeginOffsetPropertyName), jsNumber(exec, divotPoint - startOffset), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionCaretOffsetPropertyName), jsNumber(exec, divotPoint), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionEndOffsetPropertyName), jsNumber(exec, divotPoint + endOffset), ReadOnly | DontDelete);
    return exception;
}

JSNotAnObjectErrorStub* createNotAnObjectErrorStub(ExecState* exec, bool isNull)
{
    return new (exec) JSNotAnObjectErrorStub(exec, isNull);
}

JSObject* createNotAnObjectError(ExecState* exec, JSNotAnObjectErrorStub* error, unsigned bytecodeOffset, CodeBlock* codeBlock)
{
    // Both op_construct and op_instanceof require a use of op_get_by_id to get
    // the prototype property from an object. The exception messages for exceptions
    // thrown by these instances op_get_by_id need to reflect this.
    OpcodeID followingOpcodeID;
    if (codeBlock->getByIdExceptionInfoForBytecodeOffset(exec, bytecodeOffset, followingOpcodeID)) {
        ASSERT(followingOpcodeID == op_construct || followingOpcodeID == op_instanceof);
        if (followingOpcodeID == op_construct)
            return createNotAConstructorError(exec, error->isNull() ? jsNull() : jsUndefined(), bytecodeOffset, codeBlock);
        return createInvalidParamError(exec, "instanceof", error->isNull() ? jsNull() : jsUndefined(), bytecodeOffset, codeBlock);
    }

    int startOffset = 0;
    int endOffset = 0;
    int divotPoint = 0;
    int line = codeBlock->expressionRangeForBytecodeOffset(exec, bytecodeOffset, divotPoint, startOffset, endOffset);
    UString errorMessage = createErrorMessage(exec, codeBlock, line, divotPoint - startOffset, divotPoint, error->isNull() ? jsNull() : jsUndefined(), "not an object");
    JSObject* exception = Error::create(exec, TypeError, errorMessage, line, codeBlock->ownerExecutable()->sourceID(), codeBlock->ownerExecutable()->sourceURL());
    exception->putWithAttributes(exec, Identifier(exec, expressionBeginOffsetPropertyName), jsNumber(exec, divotPoint - startOffset), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionCaretOffsetPropertyName), jsNumber(exec, divotPoint), ReadOnly | DontDelete);
    exception->putWithAttributes(exec, Identifier(exec, expressionEndOffsetPropertyName), jsNumber(exec, divotPoint + endOffset), ReadOnly | DontDelete);
    return exception;
}

JSValue throwOutOfMemoryError(ExecState* exec)
{
    return throwError(exec, GeneralError, "Out of memory");
}

} // namespace JSC
