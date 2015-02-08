

#ifndef WorldContextHandle_h
#define WorldContextHandle_h

#include "SharedPersistent.h"

#include <v8.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class V8Proxy;

enum WorldToUse { UseMainWorld, UseCurrentWorld };

class WorldContextHandle {
public:
    WorldContextHandle(WorldToUse);
    v8::Local<v8::Context> adjustedContext(V8Proxy*) const;

private:
    WorldToUse m_worldToUse;
    RefPtr<SharedPersistent<v8::Context> > m_context;
};

} // namespace WebCore

#endif // WorldContextHandle_h
