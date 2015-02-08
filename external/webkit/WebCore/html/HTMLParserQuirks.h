

#ifndef HTMLParserQuirks_h
#define HTMLParserQuirks_h

#include <wtf/Noncopyable.h>

namespace WebCore {

class AtomicString;
class Node;

class HTMLParserQuirks : public Noncopyable {
public:
    HTMLParserQuirks() { }
    virtual ~HTMLParserQuirks() { }

    virtual void reset() = 0;

    virtual bool shouldInsertNode(Node* parent, Node* newNode) = 0;
    virtual bool shouldPopBlock(const AtomicString& tagNameOnStack, const AtomicString& tagNameToPop) = 0;
};

} // namespace WebCore

#endif // HTMLParserQuirks_h
