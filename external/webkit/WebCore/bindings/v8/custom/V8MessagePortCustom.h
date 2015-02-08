

#ifndef V8MessagePortCustom_h
#define V8MessagePortCustom_h

#include <v8.h>

#include "MessagePort.h"

namespace WebCore {

    // Helper function which pulls the values out of a JS sequence and into a MessagePortArray.
    // Also validates the elements per sections 4.1.13 and 4.1.15 of the WebIDL spec and section 8.3.3 of the HTML5 spec and generates exceptions as appropriate.
    // Returns true if the array was filled, or false if the passed value was not of an appropriate type.
    bool getMessagePortArray(v8::Local<v8::Value>, MessagePortArray&);

} // namespace WebCore

#endif // V8MessagePortCustom_h
