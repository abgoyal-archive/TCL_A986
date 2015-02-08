

#include "config.h"
#include "Profiler.h"

#include "CommonIdentifiers.h"
#include "CallFrame.h"
#include "CodeBlock.h"
#include "JSFunction.h"
#include "JSGlobalObject.h"
#include "Nodes.h"
#include "Profile.h"
#include "ProfileGenerator.h"
#include "ProfileNode.h"
#include <stdio.h>

namespace JSC {

static const char* GlobalCodeExecution = "(program)";
static const char* AnonymousFunction = "(anonymous function)";
static unsigned ProfilesUID = 0;

static CallIdentifier createCallIdentifierFromFunctionImp(ExecState*, JSFunction*);

Profiler* Profiler::s_sharedProfiler = 0;
Profiler* Profiler::s_sharedEnabledProfilerReference = 0;

Profiler* Profiler::profiler()
{
    if (!s_sharedProfiler)
        s_sharedProfiler = new Profiler();
    return s_sharedProfiler;
}   

void Profiler::startProfiling(ExecState* exec, const UString& title)
{
    ASSERT_ARG(title, !title.isNull());

    // Check if we currently have a Profile for this global ExecState and title.
    // If so return early and don't create a new Profile.
    ExecState* globalExec = exec ? exec->lexicalGlobalObject()->globalExec() : 0;

    for (size_t i = 0; i < m_currentProfiles.size(); ++i) {
        ProfileGenerator* profileGenerator = m_currentProfiles[i].get();
        if (profileGenerator->originatingGlobalExec() == globalExec && profileGenerator->title() == title)
            return;
    }

    s_sharedEnabledProfilerReference = this;
    RefPtr<ProfileGenerator> profileGenerator = ProfileGenerator::create(title, exec, ++ProfilesUID);
    m_currentProfiles.append(profileGenerator);
}

PassRefPtr<Profile> Profiler::stopProfiling(ExecState* exec, const UString& title)
{
    ExecState* globalExec = exec ? exec->lexicalGlobalObject()->globalExec() : 0;
    for (ptrdiff_t i = m_currentProfiles.size() - 1; i >= 0; --i) {
        ProfileGenerator* profileGenerator = m_currentProfiles[i].get();
        if (profileGenerator->originatingGlobalExec() == globalExec && (title.isNull() || profileGenerator->title() == title)) {
            profileGenerator->stopProfiling();
            RefPtr<Profile> returnProfile = profileGenerator->profile();

            m_currentProfiles.remove(i);
            if (!m_currentProfiles.size())
                s_sharedEnabledProfilerReference = 0;
            
            return returnProfile;
        }
    }

    return 0;
}

static inline void dispatchFunctionToProfiles(const Vector<RefPtr<ProfileGenerator> >& profiles, ProfileGenerator::ProfileFunction function, const CallIdentifier& callIdentifier, unsigned currentProfileTargetGroup)
{
    for (size_t i = 0; i < profiles.size(); ++i) {
        if (profiles[i]->profileGroup() == currentProfileTargetGroup || !profiles[i]->originatingGlobalExec())
            (profiles[i].get()->*function)(callIdentifier);
    }
}

void Profiler::willExecute(ExecState* exec, JSValue function)
{
    ASSERT(!m_currentProfiles.isEmpty());

    dispatchFunctionToProfiles(m_currentProfiles, &ProfileGenerator::willExecute, createCallIdentifier(exec, function, "", 0), exec->lexicalGlobalObject()->profileGroup());
}

void Profiler::willExecute(ExecState* exec, const UString& sourceURL, int startingLineNumber)
{
    ASSERT(!m_currentProfiles.isEmpty());

    CallIdentifier callIdentifier = createCallIdentifier(exec, JSValue(), sourceURL, startingLineNumber);

    dispatchFunctionToProfiles(m_currentProfiles, &ProfileGenerator::willExecute, callIdentifier, exec->lexicalGlobalObject()->profileGroup());
}

void Profiler::didExecute(ExecState* exec, JSValue function)
{
    ASSERT(!m_currentProfiles.isEmpty());

    dispatchFunctionToProfiles(m_currentProfiles, &ProfileGenerator::didExecute, createCallIdentifier(exec, function, "", 0), exec->lexicalGlobalObject()->profileGroup());
}

void Profiler::didExecute(ExecState* exec, const UString& sourceURL, int startingLineNumber)
{
    ASSERT(!m_currentProfiles.isEmpty());

    dispatchFunctionToProfiles(m_currentProfiles, &ProfileGenerator::didExecute, createCallIdentifier(exec, JSValue(), sourceURL, startingLineNumber), exec->lexicalGlobalObject()->profileGroup());
}

CallIdentifier Profiler::createCallIdentifier(ExecState* exec, JSValue functionValue, const UString& defaultSourceURL, int defaultLineNumber)
{
    if (!functionValue)
        return CallIdentifier(GlobalCodeExecution, defaultSourceURL, defaultLineNumber);
    if (!functionValue.isObject())
        return CallIdentifier("(unknown)", defaultSourceURL, defaultLineNumber);
    if (asObject(functionValue)->inherits(&JSFunction::info)) {
        JSFunction* function = asFunction(functionValue);
        if (!function->executable()->isHostFunction())
            return createCallIdentifierFromFunctionImp(exec, function);
    }
    if (asObject(functionValue)->inherits(&InternalFunction::info))
        return CallIdentifier(static_cast<InternalFunction*>(asObject(functionValue))->name(exec), defaultSourceURL, defaultLineNumber);
    return CallIdentifier(makeString("(", asObject(functionValue)->className(), " object)"), defaultSourceURL, defaultLineNumber);
}

CallIdentifier createCallIdentifierFromFunctionImp(ExecState* exec, JSFunction* function)
{
    ASSERT(!function->isHostFunction());
    const UString& name = function->calculatedDisplayName(exec);
    return CallIdentifier(name.isEmpty() ? AnonymousFunction : name, function->jsExecutable()->sourceURL(), function->jsExecutable()->lineNo());
}

} // namespace JSC
