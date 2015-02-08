

#ifndef ScriptCachedFrameData_h
#define ScriptCachedFrameData_h

#if PLATFORM(CHROMIUM)
// We don't use WebKit's page caching, so this implementation is just a stub.

namespace WebCore {

class Frame;
class DOMWindow;

class ScriptCachedFrameData  {
public:
    ScriptCachedFrameData(Frame*) { }
    ~ScriptCachedFrameData() { }

    void restore(Frame*) { }
    void clear() { }
    DOMWindow* domWindow() const { return 0; }
};

} // namespace WebCore

#elif PLATFORM(ANDROID)
// FIXME: the right guard should be ENABLE(PAGE_CACHE). Replace with the right guard, once
// https://bugs.webkit.org/show_bug.cgi?id=35061 is fixed.
//
// On Android we do use WebKit's page cache. For now we don't support isolated worlds
// so our implementation does not take them into account.

#include "OwnHandle.h"
#include <v8.h>
#include <wtf/Noncopyable.h>

namespace WebCore {

class Frame;
class DOMWindow;

class ScriptCachedFrameData  : public Noncopyable {
public:
    ScriptCachedFrameData(Frame*);
    ~ScriptCachedFrameData() { }

    void restore(Frame*);
    void clear();
    DOMWindow* domWindow() const;

private:
    OwnHandle<v8::Object> m_global;
    OwnHandle<v8::Context> m_context;
    DOMWindow* m_domWindow;
};

} // namespace WebCore

#else
#error You need to consider whether you want Page Cache and either add a stub or a real implementation.
#endif // PLATFORM(CHROMIUM)

#endif // ScriptCachedFrameData_h
