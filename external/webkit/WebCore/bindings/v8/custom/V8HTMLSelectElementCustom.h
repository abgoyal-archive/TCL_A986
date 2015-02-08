

#ifndef V8HTMLSelectElementCustom_h
#define V8HTMLSelectElementCustom_h

#include <v8.h>

namespace WebCore {

    class HTMLSelectElement;

    v8::Handle<v8::Value> removeElement(HTMLSelectElement*, const v8::Arguments&);

} // namespace WebCore

#endif // V8HTMLSelectElementCustom_h
