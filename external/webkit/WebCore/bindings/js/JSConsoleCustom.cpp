

#include "config.h"

#include "JSConsole.h"

#include "Console.h"
#include "JavaScriptProfile.h"
#include "ScriptCallStack.h"
#include <runtime/JSArray.h>

using namespace JSC;

namespace WebCore {

#if ENABLE(JAVASCRIPT_DEBUGGER)

typedef Vector<RefPtr<JSC::Profile> > ProfilesArray;

JSValue JSConsole::profiles(ExecState* exec) const
{
    const ProfilesArray& profiles = impl()->profiles();
    MarkedArgumentBuffer list;

    ProfilesArray::const_iterator end = profiles.end();
    for (ProfilesArray::const_iterator iter = profiles.begin(); iter != end; ++iter)
        list.append(toJS(exec, iter->get()));

    return constructArray(exec, list);
}

#endif

} // namespace WebCore
