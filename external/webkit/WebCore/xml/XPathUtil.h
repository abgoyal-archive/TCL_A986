

#ifndef XPathUtil_h
#define XPathUtil_h

#if ENABLE(XPATH)

#include <wtf/Vector.h>

namespace WebCore {

    class Node;
    class String;

    namespace XPath {

        /* @return whether the given node is the root node */
        bool isRootDomNode(Node*);

        /* @return the 'string-value' of the given node as specified by http://www.w3.org/TR/xpath */
        String stringValue(Node*);

        /* @return whether the given node is a valid context node */
        bool isValidContextNode(Node*);

    }

}

#endif // ENABLE(XPATH)

#endif // XPath_Util_H
