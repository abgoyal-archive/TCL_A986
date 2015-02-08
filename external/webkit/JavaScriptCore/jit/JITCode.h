

#ifndef JITCode_h
#define JITCode_h

#include <wtf/Platform.h>

#if ENABLE(JIT)

#include "CallFrame.h"
#include "JSValue.h"
#include "MacroAssemblerCodeRef.h"
#include "Profiler.h"

namespace JSC {

    class JSGlobalData;
    class RegisterFile;

    class JITCode {
        typedef MacroAssemblerCodeRef CodeRef;
        typedef MacroAssemblerCodePtr CodePtr;
    public:
        JITCode()
        {
        }

        JITCode(const CodeRef ref)
            : m_ref(ref)
        {
        }

        bool operator !() const
        {
            return !m_ref.m_code.executableAddress();
        }

        CodePtr addressForCall()
        {
            return m_ref.m_code;
        }

        // This function returns the offset in bytes of 'pointerIntoCode' into
        // this block of code.  The pointer provided must be a pointer into this
        // block of code.  It is ASSERTed that no codeblock >4gb in size.
        unsigned offsetOf(void* pointerIntoCode)
        {
            intptr_t result = reinterpret_cast<intptr_t>(pointerIntoCode) - reinterpret_cast<intptr_t>(m_ref.m_code.executableAddress());
            ASSERT(static_cast<intptr_t>(static_cast<unsigned>(result)) == result);
            return static_cast<unsigned>(result);
        }

        // Execute the code!
        inline JSValue execute(RegisterFile* registerFile, CallFrame* callFrame, JSGlobalData* globalData, JSValue* exception)
        {
            return JSValue::decode(ctiTrampoline(m_ref.m_code.executableAddress(), registerFile, callFrame, exception, Profiler::enabledProfilerReference(), globalData));
        }

        void* start()
        {
            return m_ref.m_code.dataLocation();
        }

        size_t size()
        {
            ASSERT(m_ref.m_code.executableAddress());
            return m_ref.m_size;
        }

        ExecutablePool* getExecutablePool()
        {
            return m_ref.m_executablePool.get();
        }

        // Host functions are a bit special; they have a m_code pointer but they
        // do not individully ref the executable pool containing the trampoline.
        static JITCode HostFunction(CodePtr code)
        {
            return JITCode(code.dataLocation(), 0, 0);
        }

    private:
        JITCode(void* code, PassRefPtr<ExecutablePool> executablePool, size_t size)
            : m_ref(code, executablePool, size)
        {
        }

        CodeRef m_ref;
    };

};

#endif

#endif
