

#ifndef Entity_h
#define Entity_h

#include "ContainerNode.h"

namespace WebCore {

// FIXME: This abstract class is only here so that the JavaScript and Objective-C bindings
// can continue to be compiled.
class Entity : public ContainerNode {
public:
    String publicId() const { ASSERT_NOT_REACHED(); return String(); }
    String systemId() const { ASSERT_NOT_REACHED(); return String(); }
    String notationName() const { ASSERT_NOT_REACHED(); return String(); }

private:
    Entity() : ContainerNode(0) {}
};

} //namespace

#endif
