

#ifndef Comment_h
#define Comment_h

#include "CharacterData.h"

namespace WebCore {

class Comment : public CharacterData {
public:
    static PassRefPtr<Comment> create(Document*, const String&);

private:
    Comment(Document*, const String&);

    virtual String nodeName() const;
    virtual NodeType nodeType() const;
    virtual PassRefPtr<Node> cloneNode(bool deep);
    virtual bool isCommentNode() const { return true; }
    virtual bool childTypeAllowed(NodeType);
};

} // namespace WebCore

#endif // Comment_h
