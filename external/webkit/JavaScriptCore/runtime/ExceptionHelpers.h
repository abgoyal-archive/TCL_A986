

#ifndef ExceptionHelpers_h
#define ExceptionHelpers_h


namespace JSC {

    class CodeBlock;
    class ExecState;
    class Identifier;
    class JSGlobalData;
    class JSNotAnObjectErrorStub;
    class JSObject;
    class JSValue;
    class Node;
    struct Instruction;
    
    JSValue createInterruptedExecutionException(JSGlobalData*);
    JSValue createStackOverflowError(ExecState*);
    JSValue createTypeError(ExecState*, const char* message);
    JSValue createUndefinedVariableError(ExecState*, const Identifier&, unsigned bytecodeOffset, CodeBlock*);
    JSNotAnObjectErrorStub* createNotAnObjectErrorStub(ExecState*, bool isNull);
    JSObject* createInvalidParamError(ExecState*, const char* op, JSValue, unsigned bytecodeOffset, CodeBlock*);
    JSObject* createNotAConstructorError(ExecState*, JSValue, unsigned bytecodeOffset, CodeBlock*);
    JSValue createNotAFunctionError(ExecState*, JSValue, unsigned bytecodeOffset, CodeBlock*);
    JSObject* createNotAnObjectError(ExecState*, JSNotAnObjectErrorStub*, unsigned bytecodeOffset, CodeBlock*);
    JSValue throwOutOfMemoryError(ExecState*);

} // namespace JSC

#endif // ExceptionHelpers_h
