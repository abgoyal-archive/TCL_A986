

#ifndef Interpreter_h
#define Interpreter_h

#include "ArgList.h"
#include "FastAllocBase.h"
#include "JSCell.h"
#include "JSValue.h"
#include "JSObject.h"
#include "Opcode.h"
#include "RegisterFile.h"

#include <wtf/HashMap.h>

namespace JSC {

    class CodeBlock;
    class EvalExecutable;
    class FunctionExecutable;
    class InternalFunction;
    class JSFunction;
    class JSGlobalObject;
    class ProgramExecutable;
    class Register;
    class ScopeChainNode;
    class SamplingTool;
    struct CallFrameClosure;
    struct HandlerInfo;
    struct Instruction;
    
    enum DebugHookID {
        WillExecuteProgram,
        DidExecuteProgram,
        DidEnterCallFrame,
        DidReachBreakpoint,
        WillLeaveCallFrame,
        WillExecuteStatement
    };

    enum { MaxMainThreadReentryDepth = 256, MaxSecondaryThreadReentryDepth = 32 };

    class Interpreter : public FastAllocBase {
        friend class JIT;
        friend class CachedCall;
    public:
        Interpreter();

        RegisterFile& registerFile() { return m_registerFile; }
        
        Opcode getOpcode(OpcodeID id)
        {
            #if HAVE(COMPUTED_GOTO)
                return m_opcodeTable[id];
            #else
                return id;
            #endif
        }

        OpcodeID getOpcodeID(Opcode opcode)
        {
            #if HAVE(COMPUTED_GOTO)
                ASSERT(isOpcode(opcode));
                return m_opcodeIDTable.get(opcode);
            #else
                return opcode;
            #endif
        }

        bool isOpcode(Opcode);
        
        JSValue execute(ProgramExecutable*, CallFrame*, ScopeChainNode*, JSObject* thisObj, JSValue* exception);
        JSValue execute(FunctionExecutable*, CallFrame*, JSFunction*, JSObject* thisObj, const ArgList& args, ScopeChainNode*, JSValue* exception);
        JSValue execute(EvalExecutable* evalNode, CallFrame* exec, JSObject* thisObj, ScopeChainNode* scopeChain, JSValue* exception);

        JSValue retrieveArguments(CallFrame*, JSFunction*) const;
        JSValue retrieveCaller(CallFrame*, InternalFunction*) const;
        void retrieveLastCaller(CallFrame*, int& lineNumber, intptr_t& sourceID, UString& sourceURL, JSValue& function) const;
        
        void getArgumentsData(CallFrame*, JSFunction*&, ptrdiff_t& firstParameterIndex, Register*& argv, int& argc);
        
        SamplingTool* sampler() { return m_sampler.get(); }

        NEVER_INLINE JSValue callEval(CallFrame*, RegisterFile*, Register* argv, int argc, int registerOffset, JSValue& exceptionValue);
        NEVER_INLINE HandlerInfo* throwException(CallFrame*&, JSValue&, unsigned bytecodeOffset, bool);
        NEVER_INLINE void debug(CallFrame*, DebugHookID, int firstLine, int lastLine);

        void dumpSampleData(ExecState* exec);
        void startSampling();
        void stopSampling();
    private:
        enum ExecutionFlag { Normal, InitializeAndReturn };

        CallFrameClosure prepareForRepeatCall(FunctionExecutable*, CallFrame*, JSFunction*, int argCount, ScopeChainNode*, JSValue* exception);
        void endRepeatCall(CallFrameClosure&);
        JSValue execute(CallFrameClosure&, JSValue* exception);

        JSValue execute(EvalExecutable*, CallFrame*, JSObject* thisObject, int globalRegisterOffset, ScopeChainNode*, JSValue* exception);

#if USE(INTERPRETER)
        NEVER_INLINE bool resolve(CallFrame*, Instruction*, JSValue& exceptionValue);
        NEVER_INLINE bool resolveSkip(CallFrame*, Instruction*, JSValue& exceptionValue);
        NEVER_INLINE bool resolveGlobal(CallFrame*, Instruction*, JSValue& exceptionValue);
        NEVER_INLINE void resolveBase(CallFrame*, Instruction* vPC);
        NEVER_INLINE bool resolveBaseAndProperty(CallFrame*, Instruction*, JSValue& exceptionValue);
        NEVER_INLINE ScopeChainNode* createExceptionScope(CallFrame*, const Instruction* vPC);

        void tryCacheGetByID(CallFrame*, CodeBlock*, Instruction*, JSValue baseValue, const Identifier& propertyName, const PropertySlot&);
        void uncacheGetByID(CodeBlock*, Instruction* vPC);
        void tryCachePutByID(CallFrame*, CodeBlock*, Instruction*, JSValue baseValue, const PutPropertySlot&);
        void uncachePutByID(CodeBlock*, Instruction* vPC);        
#endif

        NEVER_INLINE bool unwindCallFrame(CallFrame*&, JSValue, unsigned& bytecodeOffset, CodeBlock*&);

        static ALWAYS_INLINE CallFrame* slideRegisterWindowForCall(CodeBlock*, RegisterFile*, CallFrame*, size_t registerOffset, int argc);

        static CallFrame* findFunctionCallFrame(CallFrame*, InternalFunction*);

        JSValue privateExecute(ExecutionFlag, RegisterFile*, CallFrame*, JSValue* exception);

        void dumpCallFrame(CallFrame*);
        void dumpRegisters(CallFrame*);
        
        bool isCallBytecode(Opcode opcode) { return opcode == getOpcode(op_call) || opcode == getOpcode(op_construct) || opcode == getOpcode(op_call_eval); }

        void enableSampler();
        int m_sampleEntryDepth;
        OwnPtr<SamplingTool> m_sampler;

        int m_reentryDepth;

        RegisterFile m_registerFile;
        
#if HAVE(COMPUTED_GOTO)
        Opcode m_opcodeTable[numOpcodeIDs]; // Maps OpcodeID => Opcode for compiling
        HashMap<Opcode, OpcodeID> m_opcodeIDTable; // Maps Opcode => OpcodeID for decompiling
#endif
    };
    
} // namespace JSC

#endif // Interpreter_h
