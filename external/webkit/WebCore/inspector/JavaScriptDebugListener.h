

#ifndef JavaScriptDebugListener_h
#define JavaScriptDebugListener_h

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)

namespace JSC {
    class ExecState;
    class SourceCode;
    class UString;
}

namespace WebCore {

    class Frame;
    class Page;

    class JavaScriptDebugListener {
    public:
        virtual ~JavaScriptDebugListener() { }

        virtual void didParseSource(JSC::ExecState*, const JSC::SourceCode& source) = 0;
        virtual void failedToParseSource(JSC::ExecState*, const JSC::SourceCode& source, int errorLine, const JSC::UString& errorMessage) = 0;
        virtual void didPause() = 0;
        virtual void didContinue() = 0;
    };

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)

#endif // JavaScriptDebugListener_h
