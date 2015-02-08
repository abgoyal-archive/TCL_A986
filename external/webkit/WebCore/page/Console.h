

#ifndef Console_h
#define Console_h

#include "PlatformString.h"

#include "ScriptProfile.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

#if ENABLE(JAVASCRIPT_DEBUGGER)
typedef Vector<RefPtr<ScriptProfile> > ProfilesArray;
#endif

class Frame;
class Page;
class String;
class ScriptCallStack;

// Keep in sync with inspector/front-end/Console.js
enum MessageSource {
    HTMLMessageSource,
    WMLMessageSource,
    XMLMessageSource,
    JSMessageSource,
    CSSMessageSource,
    OtherMessageSource
};

enum MessageType {
    LogMessageType,
    ObjectMessageType,
    TraceMessageType,
    StartGroupMessageType,
    EndGroupMessageType,
    AssertMessageType
};

enum MessageLevel {
    TipMessageLevel,
    LogMessageLevel,
    WarningMessageLevel,
    ErrorMessageLevel,
    DebugMessageLevel
};

class Console : public RefCounted<Console> {
public:
    static PassRefPtr<Console> create(Frame* frame) { return adoptRef(new Console(frame)); }

    Frame* frame() const;
    void disconnectFrame();

    void addMessage(MessageSource, MessageType, MessageLevel, const String& message, unsigned lineNumber, const String& sourceURL);

    void debug(ScriptCallStack*);
    void error(ScriptCallStack*);
    void info(ScriptCallStack*);
    void log(ScriptCallStack*);
    void warn(ScriptCallStack*);
    void dir(ScriptCallStack*);
    void dirxml(ScriptCallStack*);
    void trace(ScriptCallStack*);
    void assertCondition(bool condition, ScriptCallStack*);
    void count(ScriptCallStack*);
    void markTimeline(ScriptCallStack*);
#if ENABLE(WML)
    String lastWMLErrorMessage() const;
#endif
#if ENABLE(JAVASCRIPT_DEBUGGER)
    void profile(const String&, ScriptCallStack*);
    void profileEnd(const String&, ScriptCallStack*);
#endif
    void time(const String&);
    void timeEnd(const String&, ScriptCallStack*);
    void group(ScriptCallStack*);
    void groupEnd();

    static bool shouldPrintExceptions();
    static void setShouldPrintExceptions(bool);

#if ENABLE(JAVASCRIPT_DEBUGGER)
    const ProfilesArray& profiles() const { return m_profiles; }
#endif

private:
    inline Page* page() const;
    void addMessage(MessageType, MessageLevel, ScriptCallStack*, bool acceptNoArguments = false);

    Console(Frame*);

    Frame* m_frame;
#if ENABLE(JAVASCRIPT_DEBUGGER)
    ProfilesArray m_profiles;
#endif
};

} // namespace WebCore

#endif // Console_h
