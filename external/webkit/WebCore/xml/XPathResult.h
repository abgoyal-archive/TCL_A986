

#ifndef XPathResult_h
#define XPathResult_h

#if ENABLE(XPATH)

#include "XPathValue.h"
#include <wtf/RefCounted.h>

namespace WebCore {

    typedef int ExceptionCode;

    class Document;
    class Node;
    class String;

    class XPathResult : public RefCounted<XPathResult> {
    public:
        enum XPathResultType {
            ANY_TYPE = 0,
            NUMBER_TYPE = 1,
            STRING_TYPE = 2,
            BOOLEAN_TYPE = 3,
            UNORDERED_NODE_ITERATOR_TYPE = 4,
            ORDERED_NODE_ITERATOR_TYPE = 5,
            UNORDERED_NODE_SNAPSHOT_TYPE = 6,
            ORDERED_NODE_SNAPSHOT_TYPE = 7,
            ANY_UNORDERED_NODE_TYPE = 8,
            FIRST_ORDERED_NODE_TYPE = 9
        };
        
        static PassRefPtr<XPathResult> create(Document* document, const XPath::Value& value) { return adoptRef(new XPathResult(document, value)); }
        ~XPathResult();
        
        void convertTo(unsigned short type, ExceptionCode&);

        unsigned short resultType() const;

        double numberValue(ExceptionCode&) const;
        String stringValue(ExceptionCode&) const;
        bool booleanValue(ExceptionCode&) const;
        Node* singleNodeValue(ExceptionCode&) const;

        bool invalidIteratorState() const;
        unsigned long snapshotLength(ExceptionCode&) const;
        Node* iterateNext(ExceptionCode&);
        Node* snapshotItem(unsigned long index, ExceptionCode&);

    private:
        XPathResult(Document*, const XPath::Value&);
        
        XPath::Value m_value;
        unsigned m_nodeSetPosition;
        XPath::NodeSet m_nodeSet; // FIXME: why duplicate the node set stored in m_value?
        unsigned short m_resultType;
        RefPtr<Document> m_document;
        unsigned m_domTreeVersion;
    };

} // namespace WebCore

#endif // ENABLE(XPATH)

#endif // XPathResult_h
