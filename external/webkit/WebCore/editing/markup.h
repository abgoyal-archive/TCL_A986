

#ifndef markup_h
#define markup_h

#include "HTMLInterchange.h"
#include "MappedAttributeEntry.h"
#include <wtf/Forward.h>
#include <wtf/Vector.h>

namespace WebCore {

    class Document;
    class DocumentFragment;
    class Node;
    class Range;
    class String;

    enum EChildrenOnly { IncludeNode, ChildrenOnly };

    PassRefPtr<DocumentFragment> createFragmentFromText(Range* context, const String& text);
    PassRefPtr<DocumentFragment> createFragmentFromMarkup(Document*, const String& markup, const String& baseURL, FragmentScriptingPermission = FragmentScriptingAllowed);
    PassRefPtr<DocumentFragment> createFragmentFromNodes(Document*, const Vector<Node*>&);

    bool isPlainTextMarkup(Node *node);

    String createMarkup(const Range*,
        Vector<Node*>* = 0, EAnnotateForInterchange = DoNotAnnotateForInterchange, bool convertBlocksToInlines = false);
    String createMarkup(const Node*, EChildrenOnly = IncludeNode, Vector<Node*>* = 0);
    
    String createFullMarkup(const Node*);
    String createFullMarkup(const Range*);

}

#endif // markup_h
