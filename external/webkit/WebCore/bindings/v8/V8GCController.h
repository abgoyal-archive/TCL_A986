

#ifndef V8GCController_h
#define V8GCController_h

#include <v8.h>

namespace WebCore {

#ifndef NDEBUG

#define GlobalHandleTypeList(V)   \
    V(PROXY)                      \
    V(NPOBJECT)                   \
    V(SCHEDULED_ACTION)           \
    V(EVENT_LISTENER)             \
    V(NODE_FILTER)                \
    V(SCRIPTINSTANCE)             \
    V(SCRIPTVALUE)                \
    V(DATASOURCE)


    // Host information of persistent handles.
    enum GlobalHandleType {
#define ENUM(name) name,
        GlobalHandleTypeList(ENUM)
#undef ENUM
    };

    class GlobalHandleInfo {
    public:
        GlobalHandleInfo(void* host, GlobalHandleType type) : m_host(host), m_type(type) { }
        void* m_host;
        GlobalHandleType m_type;
    };

#endif // NDEBUG

    class V8GCController {
    public:
        // Protect/Unprotect JS wrappers of a DOM object.
        static void gcProtect(void* domObject);
        static void gcUnprotect(void* domObject);

#ifndef NDEBUG
        // For debugging and leak detection purpose.
        static void registerGlobalHandle(GlobalHandleType, void*, v8::Persistent<v8::Value>);
        static void unregisterGlobalHandle(void*, v8::Persistent<v8::Value>);
#endif

        static void gcPrologue();
        static void gcEpilogue();

        static void checkMemoryUsage();

    private:
        // Estimate of current working set.
        static int workingSetEstimateMB;
    };

}

#endif // V8GCController_h
