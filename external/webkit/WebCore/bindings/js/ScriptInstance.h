

#ifndef ScriptInstance_h
#define ScriptInstance_h

#include "Bridge.h"
#include <wtf/RefPtr.h>

namespace WebCore {

typedef RefPtr<JSC::Bindings::Instance> ScriptInstance;
typedef PassRefPtr<JSC::Bindings::Instance> PassScriptInstance;

} // namespace WebCore

#endif // ScriptInstance_h
