

#ifndef V8HTMLAudioElementConstructor_h
#define V8HTMLAudioElementConstructor_h

#include <v8.h>

namespace WebCore {

class V8HTMLAudioElementConstructor {
public:
    static v8::Persistent<v8::FunctionTemplate> GetTemplate();
};

}

#endif // V8HTMLAudioElementConstructor_h
