

#ifndef HTMLCollection_h
#define HTMLCollection_h

#include "CollectionType.h"
#include <wtf/RefCounted.h>
#include <wtf/Forward.h>
#include <wtf/HashMap.h>
#include <wtf/Vector.h>

namespace WebCore {

class AtomicString;
class AtomicStringImpl;
class Element;
class Node;
class NodeList;
class String;

struct CollectionCache;

class HTMLCollection : public RefCounted<HTMLCollection> {
public:
    static PassRefPtr<HTMLCollection> create(PassRefPtr<Node> base, CollectionType);
    virtual ~HTMLCollection();
    
    unsigned length() const;
    
    virtual Node* item(unsigned index) const;
    virtual Node* nextItem() const;

    virtual Node* namedItem(const AtomicString& name) const;
    virtual Node* nextNamedItem(const AtomicString& name) const; // In case of multiple items named the same way

    Node* firstItem() const;

    void namedItems(const AtomicString& name, Vector<RefPtr<Node> >&) const;

    PassRefPtr<NodeList> tags(const String&);

    Node* base() const { return m_base.get(); }
    CollectionType type() const { return m_type; }

protected:
    HTMLCollection(PassRefPtr<Node> base, CollectionType, CollectionCache*);
    HTMLCollection(PassRefPtr<Node> base, CollectionType);

    CollectionCache* info() const { return m_info; }
    void resetCollectionInfo() const;

    mutable bool m_idsDone; // for nextNamedItem()

private:
    virtual Element* itemAfter(Element*) const;
    virtual unsigned calcLength() const;
    virtual void updateNameCache() const;

    bool checkForNameMatch(Element*, bool checkName, const AtomicString& name) const;

    RefPtr<Node> m_base;
    CollectionType m_type;

    mutable CollectionCache* m_info;
    mutable bool m_ownsInfo;
};

} // namespace

#endif
