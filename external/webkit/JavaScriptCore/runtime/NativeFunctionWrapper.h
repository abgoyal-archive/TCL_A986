

#ifndef NativeFunctionWrapper_h
#define NativeFunctionWrapper_h

namespace JSC {
#if ENABLE(JIT) && ENABLE(JIT_OPTIMIZE_NATIVE_CALL)
    class JSFunction;
    typedef JSFunction NativeFunctionWrapper;
#else
    class PrototypeFunction;
    typedef PrototypeFunction NativeFunctionWrapper;
#endif
}

#endif
