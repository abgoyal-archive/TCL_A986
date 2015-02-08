

#ifndef Profiler_h
#define Profiler_h

#include "Profile.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace JSC {

    class ExecState;
    class JSGlobalData;
    class JSObject;
    class JSValue;
    class ProfileGenerator;
    class UString;
    struct CallIdentifier;    

    class Profiler : public FastAllocBase {
    public:
        static Profiler** enabledProfilerReference()
        {
            return &s_sharedEnabledProfilerReference;
        }

        static Profiler* profiler(); 
        static CallIdentifier createCallIdentifier(ExecState* exec, JSValue, const UString& sourceURL, int lineNumber);

        void startProfiling(ExecState*, const UString& title);
        PassRefPtr<Profile> stopProfiling(ExecState*, const UString& title);

        void willExecute(ExecState*, JSValue function);
        void willExecute(ExecState*, const UString& sourceURL, int startingLineNumber);
        void didExecute(ExecState*, JSValue function);
        void didExecute(ExecState*, const UString& sourceURL, int startingLineNumber);

        const Vector<RefPtr<ProfileGenerator> >& currentProfiles() { return m_currentProfiles; };

    private:
        Vector<RefPtr<ProfileGenerator> > m_currentProfiles;
        static Profiler* s_sharedProfiler;
        static Profiler* s_sharedEnabledProfilerReference;
    };

} // namespace JSC

#endif // Profiler_h
