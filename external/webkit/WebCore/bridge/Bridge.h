

#ifndef Bridge_h
#define Bridge_h

#include "BridgeJSC.h"
#include <wtf/Noncopyable.h>

namespace JSC  {

namespace Bindings {

class Method : public Noncopyable {
public:
    virtual int numParameters() const = 0;

    virtual ~Method() { }
};

} // namespace Bindings

} // namespace JSC

#endif
