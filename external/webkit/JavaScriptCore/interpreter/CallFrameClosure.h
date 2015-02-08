

#ifndef CallFrameClosure_h
#define CallFrameClosure_h

namespace JSC {

struct CallFrameClosure {
    CallFrame* oldCallFrame;
    CallFrame* newCallFrame;
    JSFunction* function;
    FunctionExecutable* functionExecutable;
    JSGlobalData* globalData;
    Register* oldEnd;
    ScopeChainNode* scopeChain;
    int expectedParams;
    int providedParams;
    
    void setArgument(int arg, JSValue value)
    {
        if (arg < expectedParams)
            newCallFrame[arg - RegisterFile::CallFrameHeaderSize - expectedParams] = value;
        else
            newCallFrame[arg - RegisterFile::CallFrameHeaderSize - expectedParams - providedParams] = value;
    }
    void resetCallFrame()
    {
        newCallFrame->setScopeChain(scopeChain);
        newCallFrame->setCalleeArguments(JSValue());
        for (int i = providedParams; i < expectedParams; ++i)
            newCallFrame[i - RegisterFile::CallFrameHeaderSize - expectedParams] = jsUndefined();
    }
};

}

#endif
