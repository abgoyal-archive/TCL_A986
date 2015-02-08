

#include "config.h"
#include "Comment.h"

namespace WebCore {

inline Comment::Comment(Document* document, const String& text)
    : CharacterData(document, text, CreateOther)
{
}

PassRefPtr<Comment> Comment::create(Document* document, const String& text)
{
    return adoptRef(new Comment(document, text));
}

String Comment::nodeName() const
{
    return commentAtom.string();
}

Node::NodeType Comment::nodeType() const
{
    return COMMENT_NODE;
}

PassRefPtr<Node> Comment::cloneNode(bool /*deep*/)
{
    return create(document(), data());
}

bool Comment::childTypeAllowed(NodeType)
{
    return false;
}

} // namespace WebCore
