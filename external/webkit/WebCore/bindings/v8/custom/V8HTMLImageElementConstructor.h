

#ifndef V8HTMLImageElementConstructor_h
#define V8HTMLImageElementConstructor_h

#include <v8.h>

namespace WebCore {

class V8HTMLImageElementConstructor {
public:
    static v8::Persistent<v8::FunctionTemplate> GetTemplate();
};

}

#endif // V8HTMLImageElementConstructor_h
