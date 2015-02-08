

#ifndef Attribute_h
#define Attribute_h

#include "QualifiedName.h"

namespace WebCore {

class Attr;
class CSSStyleDeclaration;
class Element;
class NamedNodeMap;

// This has no counterpart in DOM.
// It is an internal representation of the node value of an Attr.
// The actual Attr with its value as a Text child is allocated only if needed.
class Attribute : public RefCounted<Attribute> {
    friend class Attr;
    friend class NamedNodeMap;
public:
    static PassRefPtr<Attribute> create(const QualifiedName& name, const AtomicString& value)
    {
        return adoptRef(new Attribute(name, value));
    }
    virtual ~Attribute() { }
    
    const AtomicString& value() const { return m_value; }
    const AtomicString& prefix() const { return m_name.prefix(); }
    const AtomicString& localName() const { return m_name.localName(); }
    const AtomicString& namespaceURI() const { return m_name.namespaceURI(); }
    
    const QualifiedName& name() const { return m_name; }
    
    Attr* attr() const { return m_impl; }
    PassRefPtr<Attr> createAttrIfNeeded(Element*);

    bool isNull() const { return m_value.isNull(); }
    bool isEmpty() const { return m_value.isEmpty(); }
    
    virtual PassRefPtr<Attribute> clone() const;

    // An extension to get the style information for presentational attributes.
    virtual CSSStyleDeclaration* style() const { return 0; }
    
    void setValue(const AtomicString& value) { m_value = value; }
    void setPrefix(const AtomicString& prefix) { m_name.setPrefix(prefix); }

    virtual bool isMappedAttribute() { return false; }

protected:
    Attribute(const QualifiedName& name, const AtomicString& value)
        : m_name(name), m_value(value), m_impl(0)
    {
    }
    Attribute(const AtomicString& name, const AtomicString& value)
        : m_name(nullAtom, name, nullAtom), m_value(value), m_impl(0)
    {
    }

private:
    QualifiedName m_name;
    AtomicString m_value;
    Attr* m_impl;
};

} //namespace

#endif
